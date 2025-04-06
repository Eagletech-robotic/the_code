/*
 * falcon.c
 *
 *  Created on: Nov 3, 2024
 *      Author: nboulay
 */
#include <stdio.h>
#include "nicolas.h"

#include "iot01A/top_driver.h" // c'est la dépendance au interface, rien d'autre n'est authorisé à être inclus depuis iot01A
#include "robotic/carre.h"

#include "robotic/pid.h"
#include <math.h>
#include "eaglesteward/constante.h"
#include "robotic/fusion_odo_imu.h"
#include "eaglesteward/state.h"

carre_t carre;

pid_t pid_diff;
pid_t pid_sum;

fusion_odo_imu_t fusion_odo_imu;
state_t state;

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
void autopilot(config_t * config, input_t * input, float v1, float v2, output_t* ret)
{
	float sensor1 = input->encoder1;
	float sensor2 = input->encoder2;

	float regul_sum = pid_(&pid_sum, v1+v2, sensor1 + sensor2);
	float regul_diff = pid_(&pid_diff, v1-v2, sensor1 - sensor2);

	ret->vitesse1_ratio = (regul_sum + regul_diff) / 2.0;
	ret->vitesse2_ratio = (regul_sum - regul_diff) / 2.0;
}

//  doit appeler la fonction et gérer les IOS
void nicolas_top_step(config_t* config, input_t *input, output_t* output ) {
	//gestion de la position
	float delta_x_m = 0.0f;
	float delta_y_m = 0.0f;
	float delta_theta_deg = 0.0f;
	const float alpha_orientation_ratio = 0.0f;
	fusion_odo_imu_fuse(&fusion_odo_imu,
			input->ins.accel_x, input->ins.accel_x, input->ins.yaw,
			input->encoder1, input->encoder2, config->time_step_ms/1000.0,
			state.theta_deg,
			&delta_x_m, &delta_y_m, &delta_theta_deg,
			alpha_orientation_ratio,
			TICKS_PER_REV, WHEEL_CIRCUMFERENCE_M, WHEEL_BASE_M);

	state.x_m += delta_x_m;
	state.y_m += delta_y_m;
	state.theta_deg += delta_theta_deg;
	print_state(&state);

	//gestion du jack
	if(!input->is_jack_gone) {
		return;
	}

	//gestiond de la trajectoire
	carre_in_loop(&carre, output);
	//carre_in_loop_with_heading(&carre, state.theta_deg,output);

	// asservissement en vitesse
	output_t ret;
	autopilot(config, input, output->vitesse1_ratio, output->vitesse2_ratio, &ret);
	output->vitesse1_ratio=ret.vitesse1_ratio;
	output->vitesse2_ratio=ret.vitesse2_ratio;
}

void nicolas_top_init(config_t* config) {
	config->time_step_ms = 4; // il faudrait 250hz, les get par I2C sont trop lent
	printf("cycle : %i ms\r\n",config->time_step_ms);
	carre_init(&carre, config->time_step_ms / 1000.0);
	autopilot_init(config);
	fusion_odo_imu_init(&fusion_odo_imu);
}

//TODO :
// connectivité externe en bluetooth
// prévoir les commandes de servo
