/*
 * falcon.c
 *
 *  Created on: Nov 3, 2024
 *      Author: nboulay
 */
#include "eaglesteward/guidance/nicolas.hpp"

#include <math.h>
#include <stdio.h>

#include "eaglesteward/behaviortree.hpp"
#include "eaglesteward/constante.hpp"
#include "eaglesteward/guidance/thibault.hpp"
#include "eaglesteward/pelle.hpp"
#include "eaglesteward/state.hpp"
#include "iot01A/top_driver.h" // c'est la dépendance au interface, rien d'autre n'est authorisé à être inclus depuis iot01A
#include "robotic/angle.h"
#include "robotic/carre.h"
#include "robotic/controller_stanley.h"
#include "robotic/fusion_odo_imu.h"
#include "robotic/pid.h"
#include "utils/myprintf.hpp"

carre_t carre;

pid_t pid_diff;
pid_t pid_sum;

state_t state;

//// Autopilote

float curve(float v1, float v2) { return (v1 - v2) / (v1 + v2); }

void autopilot_init(config_t *config) {
    pid_init(&pid_diff);
    pid_tune(&pid_diff, 0.00033, 0.0000000, 0.000001); // avec ki =0 est le seul moyen d'avoir R inférieur à 1% d'erreur
                                                       // (autour de 12-15v), ki fait diverger
    pid_limits(&pid_diff, -2.0, 2.0);
    float f = 1000.0 / config->time_step_ms;
    pid_frequency(&pid_diff, f);

    pid_init(&pid_sum);
    pid_tune(&pid_sum, 0.0003, 0.0000000, 0.000001); // ki est toujours trop instable au démarrage.
    pid_limits(&pid_sum, -2.0, 2.0);
    pid_frequency(&pid_sum, f);
}

// pour éviter de créer une nouvelle structure et pour évite de perdre le flux d'information, il faut envoyer un ouput
// vierge et recopier les données utiles. Cela évite de se perdre dans les mise à jours de valeurs
void autopilot(config_t *config, input_t *input, float v_gauche_m_s, float v_droite_m_s, output_t *ret) {
    float sensor_left = input->encoder_left;
    float sensor_right = input->encoder_right;

    // 72000/(3.14*0.069)/250
    float ticks_per_m = (config->time_step_ms / 1000.0f) * TICKS_PER_REV / (WHEEL_CIRCUMFERENCE_M); //~1330
    float ratio_gauche = v_gauche_m_s * ticks_per_m;
    float ratio_droite = v_droite_m_s * ticks_per_m;

    float regul_sum = pid_(&pid_sum, ratio_droite + ratio_gauche, sensor_right + sensor_left);
    float regul_diff = pid_(&pid_diff, ratio_droite - ratio_gauche, sensor_right - sensor_left);

    ret->motor_left_ratio = (regul_sum - regul_diff) / 2.0;
    ret->motor_right_ratio = (regul_sum + regul_diff) / 2.0;
}

// --- Filtre de TOF

float tof_filter(float sensors, float val) {
    if (sensors == 0.0f || sensors == 2.0f) {
        return val;
    }

    return 0.7f * val + (0.3f * sensors);
}

//// Comportement

Status gotoTarget(float start_x_m, float start_y_m, float target_x_m, float target_y_m, float next_x_m, float next_y_m,
                  int target, input_t *input, output_t *output, state_t *state) {
    if (state->target != target) {
        return Status::SUCCESS;
    }
    myprintf("B%d\r\n", state->target);
    //	int isArrived = stanley_controller(
    //	    state->x_m, state->y_m, state->theta_deg,
    //	    start_x_m, start_y_m,
    //	    target_x_m, target_y_m,
    //	    next_y_m, next_y_m,
    //	    1000.0f, //Vmax
    //	    500.0f,  // Wmax
    //	    1.0f,   // k
    //	    WHEEL_BASE_M,
    //	    0.1f, // arrivalThreshold avant virage
    //	    &output->motor_left_ratio,
    //	    &output->motor_right_ratio
    //	);
    int isArrived = controller_pid(state->x_m, state->y_m, state->theta_deg, target_x_m, target_y_m, 0.8f, WHEEL_BASE_M,
                                   0.08, &output->motor_left_ratio, &output->motor_right_ratio);
    if (isArrived) {
        state->target++;
        return Status::SUCCESS;
    }
    return Status::RUNNING;
}

// execution une fois par cycle de tout l'arbre
void infinite_rectangle(config_t *config, input_t *input, output_t *output, state_t *state) {
    auto seq =
        sequence([](input_t *input, output_t *output,
                    state_t *state) { return gotoTarget(0.0, 0.0, 0.6, 0.0, 0.6, 0.6, 0, input, output, state); },
                 [](input_t *input, output_t *output, state_t *state) {
                     return gotoTarget(0.6, 0.0, 0.6, 0.6, 0.0, 0.6, 1, input, output, state);
                 },
                 [](input_t *input, output_t *output, state_t *state) {
                     return gotoTarget(0.6, 0.6, 0.0, 0.6, 0.0, 0.0, 2, input, output, state);
                 },
                 [](input_t *input, output_t *output, state_t *state) {
                     return gotoTarget(0.0, 0.6, 0.0, 0.0, 0.6, 0.0, 3, input, output, state);
                 },
                 [](input_t *, output_t *, state_t *state) {
                     state->target = 0;
                     return Status::SUCCESS;
                 });

    seq(input, output, state);
}

void calcul_position(state_t *state, input_t *input, config_t *config) {
    // gestion de la position
    float delta_x_m = 0.0f;
    float delta_y_m = 0.0f;
    float delta_theta_deg = 0.0f;
    const float alpha_orientation_ratio = 0.5f;
    // O.O -> IMU seul
    fusion_odo_imu_fuse(input->imu_accel_x_mss, input->imu_accel_y_mss, input->delta_yaw_deg, input->encoder_left,
                        input->encoder_right, config->time_step_ms / 1000.0, state->theta_deg, &delta_x_m, &delta_y_m,
                        &delta_theta_deg, alpha_orientation_ratio, TICKS_PER_REV, WHEEL_CIRCUMFERENCE_M, WHEEL_BASE_M);
    state->x_m += delta_x_m;
    state->y_m += delta_y_m;
    state->theta_deg += delta_theta_deg;
    state->theta_deg = angle_normalize_deg(state->theta_deg);
    print_state(state);
}

//  doit appeler la fonction et gérer les IOS
void nicolas_top_step(config_t *config, input_t *input, output_t *output) {
    myprintf("\x1B[2J"); // efface l'écran de debug
    state.filtered_tof_m = tof_filter(input->tof_m, state.filtered_tof_m);
    // gestion de la position
    calcul_position(&state, input, config);

    // gestion de la trajectoire
    // carre_in_loop(&carre, output); // simpliste
    infinite_rectangle(config, input, output, &state);
    pelle_in(output);
    // thibault_top_step_bridge(input, &state, output);

    // myprintf("O %.2f %.2f\n\r", output->motor_left_ratio, output->motor_right_ratio);

    // asservissement en vitesse
    output_t ret; // vitesse en m/s converti en ticks/cycle
    autopilot(config, input, output->motor_left_ratio, output->motor_right_ratio, &ret);
    output->motor_left_ratio = ret.motor_left_ratio;
    output->motor_right_ratio = ret.motor_right_ratio;

    // gestion du jack / debug
    if (!input->is_jack_gone) {
        output->motor_left_ratio = 0;
        output->motor_right_ratio = 0;
        if (input->blue_button) {
            pelle_out(output);
        } else {
            pelle_in(output);
        }
        return;
    }
}

void nicolas_top_init(config_t *config) {
    config->time_step_ms = 4; // il faudrait 250hz, les get par I2C sont trop lent
    printf("cycle : %i ms\r\n", config->time_step_ms);
    carre_init(&carre, config->time_step_ms / 1000.0);
    autopilot_init(config);
    thibault_top_init(config);
}

// TODO :
//  connectivité externe en bluetooth
//  prévoir les commandes de servo
