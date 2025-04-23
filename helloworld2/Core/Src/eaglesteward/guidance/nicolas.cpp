#include "eaglesteward/guidance/nicolas.hpp"

#include <math.h>
#include <stdio.h>

#include "eaglesteward/behaviortree.hpp"
#include "eaglesteward/motor.hpp"
#include "eaglesteward/pelle.hpp"
#include "eaglesteward/state.hpp"
#include "eaglesteward/tof.hpp"
#include "robotic/angle.hpp"
#include "robotic/carre.hpp"
#include "robotic/controller_stanley.hpp"
#include "robotic/fusion_odo_imu.hpp"
#include "eaglesteward/robot_constants.hpp"
#include "utils/myprintf.hpp"

carre_t carre;
state_t nicolas_state;

//// Comportement

Status gotoTarget(float start_x_m, float start_y_m, float target_x_m, float target_y_m, float next_x_m, float next_y_m,
                  int target, input_t *input, output_t *output, state_t *func_state) {
    if (func_state->target != target) {
        return Status::SUCCESS;
    }
    myprintf("B%d\r\n", func_state->target);
    //	int isArrived = stanley_controller(
    //	    func_state->x_m, func_state->y_m, func_state->theta_deg,
    //	    start_x_m, start_y_m,
    //	    target_x_m, target_y_m,
    //	    next_y_m, next_y_m,
    //	    1000.0f, //Vmax
    //	    500.0f,  // Wmax
    //	    1.0f,   // k
    //	    WHEELBASE_M,
    //	    0.1f, // arrivalThreshold avant virage
    //	    &output->motor_left_ratio,
    //	    &output->motor_right_ratio
    //	);
    int isArrived = controller_pid(func_state->x_m, func_state->y_m, func_state->theta_deg, target_x_m, target_y_m,
                                   0.8f, WHEELBASE_M, 0.08, &output->motor_left_ratio, &output->motor_right_ratio);
    if (isArrived) {
        func_state->target++;
        return Status::SUCCESS;
    }
    return Status::RUNNING;
}

// execution une fois par cycle de tout l'arbre
void infinite_rectangle(config_t *config, input_t *input, output_t *output, state_t *func_state) {
    auto seq = sequence(
        [](input_t *lambda_input, output_t *lambda_output, state_t *state) {
            return gotoTarget(0.0, 0.0, 0.6, 0.0, 0.6, 0.6, 0, lambda_input, lambda_output, state);
        },
        [](input_t *lambda_input, output_t *lambda_output, state_t *state) {
            return gotoTarget(0.6, 0.0, 0.6, 0.6, 0.0, 0.6, 1, lambda_input, lambda_output, state);
        },
        [](input_t *lambda_input, output_t *lambda_output, state_t *state) {
            return gotoTarget(0.6, 0.6, 0.0, 0.6, 0.0, 0.0, 2, lambda_input, lambda_output, state);
        },
        [](input_t *lambda_input, output_t *lambda_output, state_t *state) {
            return gotoTarget(0.0, 0.6, 0.0, 0.0, 0.6, 0.0, 3, lambda_input, lambda_output, state);
        },
        [](input_t *, output_t *, state_t *state) {
            state->target = 0;
            return Status::SUCCESS;
        });

    seq(input, output, func_state);
}

void calcul_position(state_t *state, input_t *input, config_t *config) {
    // gestion de la position
    float delta_x_m = 0.0f;
    float delta_y_m = 0.0f;
    float delta_theta_deg = 0.0f;
    const float alpha_orientation_ratio = 0.5f;
    // O.O -> IMU seul
    fusion_odo_imu_fuse(input->imu_accel_x_mss, input->imu_accel_y_mss, input->delta_yaw_deg, input->encoder_left,
                        input->encoder_right, config->time_step_s, state->theta_deg, &delta_x_m, &delta_y_m,
                        &delta_theta_deg, alpha_orientation_ratio, TICKS_PER_REV, WHEEL_CIRCUMFERENCE_M, WHEELBASE_M);
    state->x_m += delta_x_m;
    state->y_m += delta_y_m;
    state->theta_deg += delta_theta_deg;
    state->theta_deg = angle_normalize_deg(state->theta_deg);
    print_state(state);
}

//  doit appeler la fonction et gérer les IOS
void nicolas_top_step(config_t *config, input_t *input, output_t *output) {
    myprintf("\x1B[2J"); // efface l'écran de debug
    nicolas_state.filtered_tof_m = tof_filter(nicolas_state, input->tof_m);
    // gestion de la position
    calcul_position(&nicolas_state, input, config);

    // gestion de la trajectoire
    // carre_in_loop(&carre, output); // simpliste
    infinite_rectangle(config, input, output, &nicolas_state);
    pelle_in(output);

    // myprintf("O %.2f %.2f\n\r", output->motor_left_ratio, output->motor_right_ratio);

    // asservissement en vitesse
    motor_calculate_ratios(*config, nicolas_state, *input, output->motor_left_ratio, output->motor_right_ratio,
                           output->motor_left_ratio, output->motor_right_ratio);

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
    config->time_step_s = 0.004f; // il faudrait 250hz, les get par I2C sont trop lent
    printf("cycle : %.0f ms\r\n", config->time_step_s * 1000.0);
    carre_init(&carre, config->time_step_s);
    motor_init(*config, nicolas_state);
    state_init(&nicolas_state);

}

// TODO :
//  connectivité externe en bluetooth
//  prévoir les commandes de servo
