/*
 * falcon.c
 *
 *  Created on: Nov 3, 2024
 *      Author: nboulay
 */
#include <stdio.h>
#include "eaglesteward/nicolas.h"

#include "iot01A/top_driver.h" // c'est la dépendance au interface, rien d'autre n'est authorisé à être inclus depuis iot01A
#include "robotic/carre.h"

#include "robotic/pid.h"
#include <math.h>
#include "eaglesteward/constante.h"
#include "robotic/fusion_odo_imu.h"
#include "eaglesteward/state.h"
#include "robotic/controller_stanley.h"
#include "eaglesteward/behaviortree.h"
#include "robotic/myprintf.h"
#include "eaglesteward/pelle.h"
#include "eaglesteward/thibault.hpp"

carre_t carre;

pid_t pid_diff;
pid_t pid_sum;

state_t state;

//// Autopilote

float curve(float v1, float v2) {
	return (v1-v2) / (v1+v2);
}

void autopilot_init(config_t * config) {
	pid_init(&pid_diff);
	pid_tune(&pid_diff, 0.00033  , 0.0000000, 0.000001); // avec ki =0 est le seul moyen d'avoir R inférieur à 1% d'erreur (autour de 12-15v), ki fait diverger
	pid_limits(&pid_diff, -2.0, 2.0);
	float f = 1000.0 / config->time_step_ms;
	pid_frequency(&pid_diff, f);

	pid_init(&pid_sum);
	pid_tune(&pid_sum, 0.0003 , 0.0000000, 0.000001); //ki est toujours trop instable au démarrage.
	pid_limits(&pid_sum, -2.0, 2.0);
	pid_frequency(&pid_sum, f);

}

// pour éviter de créer une nouvelle structure et pour évite de perdre le flux d'information, il faut envoyer un ouput vierge
// et recopier les données utiles. Cela évite de se perdre dans les mise à jours de valeurs
void autopilot(config_t * config, input_t * input, float v1_m_s, float v2_m_s, output_t* ret)
{
	float sensor1 = input->encoder1;
	float sensor2 = input->encoder2;

	//72000/(3.14*0.069)/250
	float ticks_per_m = (config->time_step_ms/ 1000.0f) * TICKS_PER_REV /(WHEEL_CIRCUMFERENCE_M) ; //~1330
	float v1 = v1_m_s * ticks_per_m;
	float v2 = v2_m_s * ticks_per_m;

	float regul_sum = pid_(&pid_sum, v1+v2, sensor1 + sensor2);
	float regul_diff = pid_(&pid_diff, v1-v2, sensor1 - sensor2);

	ret->vitesse1_ratio = (regul_sum + regul_diff) / 2.0;
	ret->vitesse2_ratio = (regul_sum - regul_diff) / 2.0;
}

// --- Filtre de TOF

float tof_filter(float sensors, float val) {
	if (sensors == 0.0f || sensors == 2.0f) {
		return val;
	}

	return 0.7f * val + (0.3f *sensors);
}

//// Comportement

Status gotoTarget(float start_x_m, float start_y_m,
			float target_x_m, float target_y_m,
			float next_x_m, float next_y_m,
			int target,
			input_t* input, output_t* output, state_t* state) {
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
//	    &output->vitesse1_ratio, // moteur droit
//		&output->vitesse2_ratio
//	);
	int isArrived = controller_pid(
		state->x_m, state->y_m, state->theta_deg,
		target_x_m, target_y_m,
	    0.8f,
		WHEEL_BASE_M,
	    0.08,
		&output->vitesse1_ratio, // moteur droit
		&output->vitesse2_ratio
	);
	if(isArrived) {
		state->target++;
		return Status::SUCCESS;
	}
	return Status::RUNNING;
}

// execution une fois par cycle de tout l'arbre
void infinite_rectangle(config_t* config, input_t *input, output_t* output, state_t * state ) {
    auto seq = sequence(
    		[](input_t* input, output_t* output, state_t* state) { return gotoTarget(0.0, 0.0,  0.6, 0.0,  0.6, 0.6,  0, input, output, state);},
			[](input_t* input, output_t* output, state_t* state) { return gotoTarget(0.6, 0.0,  0.6, 0.6,  0.0, 0.6,  1, input, output, state);},
			[](input_t* input, output_t* output, state_t* state) { return gotoTarget(0.6, 0.6,  0.0, 0.6,  0.0, 0.0,  2, input, output, state);},
			[](input_t* input, output_t* output, state_t* state) { return gotoTarget(0.0, 0.6,  0.0, 0.0,  0.6, 0.0,  3, input, output, state);},
			[](input_t*, output_t*, state_t* state) { state->target = 0; return Status::SUCCESS;}
    );

   seq(input, output, state);
}

float normalize_angle_deg(float angle_deg)
{
    // Remet dans [-360, 360] (fmod renvoie un résultat dans (-360, 360], sauf si angle_deg est un multiple exact de 360)
    angle_deg = fmodf(angle_deg, 360.0f);

    // Maintenant, on s'assure que c'est dans [-180, 180)
    if (angle_deg >= 180.0f) {
        angle_deg -= 360.0f;
    } else if (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

void calcul_position(state_t *state, input_t *input, config_t *config) {
	//gestion de la position
	float delta_x_m = 0.0f;
	float delta_y_m = 0.0f;
	float delta_theta_deg = 0.0f;
	const float alpha_orientation_ratio = 0.5f;
	// O.O -> IMU seul
	fusion_odo_imu_fuse(
			input->imu_accel_x_mss, input->imu_accel_y_mss, input->delta_yaw_deg,
			input->encoder2, input->encoder1, // gauche , droite
			config->time_step_ms / 1000.0, state->theta_deg,
			&delta_x_m, &delta_y_m, &delta_theta_deg,
			alpha_orientation_ratio, TICKS_PER_REV, WHEEL_CIRCUMFERENCE_M, WHEEL_BASE_M);
	state->x_m += delta_x_m;
	state->y_m += delta_y_m;
	state->theta_deg += delta_theta_deg;
	state->theta_deg = normalize_angle_deg (state->theta_deg);
	print_state(state);
}

//  doit appeler la fonction et gérer les IOS
void nicolas_top_step(config_t* config, input_t *input, output_t* output ) {
	myprintf("\x1B[2J"); // efface l'écran de debug
	state.filtered_tof_m = tof_filter(input->tof_m, state.filtered_tof_m);
	//gestion de la position
	calcul_position(&state, input, config);

	//gestion de la trajectoire
	//carre_in_loop(&carre, output); // simpliste
	//infinite_rectangle(config, input, output, &state);
	pelle_in(output);
	thibault_top_step_bridge(input, &state, output);

	//myprintf("O %.2f %.2f\n\r", output->vitesse1_ratio,output->vitesse2_ratio);

	// asservissement en vitesse
	output_t ret; // vitesse en m/s converti en ticks/cycle
	autopilot(config, input, output->vitesse1_ratio, output->vitesse2_ratio, &ret);
	output->vitesse1_ratio=ret.vitesse1_ratio; // roue droite
	output->vitesse2_ratio=ret.vitesse2_ratio; // roue gauche

	//gestion du jack / debug
	if(!input->is_jack_gone) {
		output->vitesse1_ratio=0;
		output->vitesse2_ratio=0;
		if(input->blue_button) {
			pelle_out(output);
		} else {
			pelle_in(output);
		}
		return;
	}
}

void nicolas_top_init(config_t* config) {
	config->time_step_ms = 4; // il faudrait 250hz, les get par I2C sont trop lent
	printf("cycle : %i ms\r\n",config->time_step_ms);
	carre_init(&carre, config->time_step_ms / 1000.0);
	autopilot_init(config);
	thibault_top_init(config);
}

//TODO :
// connectivité externe en bluetooth
// prévoir les commandes de servo
