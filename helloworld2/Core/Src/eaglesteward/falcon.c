/*
 * falcon.c
 *
 *  Created on: Nov 3, 2024
 *      Author: nboulay
 */

#include "iot01A/top_driver.h" // c'est la dépendance au interface, rien d'autre n'est authorisé à être inclus depuis iot01A
#include "robotic/carre.h"
#include <stdio.h>
#include "robotic/pid.h"

carre_t carre;
//pid_t pid_diff;
pid_t pid_sum;
pid_t pid_curve;

void top_init(config_t* config) {
	config->time_step_ms = 1;
	printf(":: %i\r\n",config->time_step_ms);
	carre_init(&carre, config->time_step_ms / 1000.0);
//	pid_init(&pid_diff);
//	//pid_tune(&pid,    0.000132, 0.00006, 0.000000165); // Ku=0.00022 Tu = 110 ms
//	pid_tune(&pid_diff, 0.000132  , 0.0000000, 0.00001); // avec ki =0 est le seul moyen d'avoir R inférieur à 1% d'erreur (autour de 12-15v)
//	pid_limits(&pid_diff, -2.0, 2.0);
//
	pid_init(&pid_sum);
	pid_tune(&pid_sum, 0.00015 , 0.0000000, 0.000001); //ki est toujours trop instable au démarrage.
	pid_limits(&pid_sum, -2.0, 2.0);

	pid_init(&pid_curve);
	pid_tune(&pid_curve, .90 , 0.005, 0.1);
	pid_limits(&pid_curve, -10000.0, 10000.0);

}

float curve(float v1, float v2) {
	return (v1-v2) / (v1+v2);
}

// top_loop doit appeler la fonction et gérer les IOS
void top_step(config_t* config, input_t *input, output_t* output ) {
	carre_in_loop(&carre, output);
	const float ratio_speed_sensor = 0.0002;
	//output->vitesse1_ratio = 1000.0 ;
	//output->vitesse2_ratio = 4000.0 ; // àdroite dans le sens de la marche
	float r_ = (output->vitesse1_ratio -output->vitesse2_ratio) /(output->vitesse1_ratio + output->vitesse2_ratio);
	//float sensor = input->encoder1 ;//- input->encoder2;œ
	//float cmd = output->vitesse1_ratio ;//- output->vitesse2_ratio;

	//pid_command(&pid,cmd);
	//float regul = pid_compute(&pid,sensor);
//	printf("  : %f %f\r\n", cmd, regul);

	float sensor1 = input->encoder1;
	float sensor2 = input->encoder2;

//	float cmd_diff = output->vitesse1_ratio - output->vitesse2_ratio;
//	float sensor_diff = sensor1 - sensor2;
//	float regul_diff = pid_(&pid_diff, cmd_diff, sensor_diff);
//	pid_print(&pid_diff);

	float cmd_sum = output->vitesse1_ratio + output->vitesse2_ratio;
	float sensor_sum = sensor1 + sensor2;
	float regul_sum = pid_(&pid_sum, cmd_sum, sensor_sum);
	//pid_print(&pid_sum);
	//output->vitesse1_ratio =( (regul_sum + regul_diff) / 2.0);
		//output->vitesse2_ratio = ((regul_sum - regul_diff) / 2.0);

	float cmd_curve = curve(output->vitesse1_ratio, output->vitesse2_ratio);
	if (sensor1+sensor2 != .0) {
		float sensor_curve = curve(sensor1, sensor2);
		float regul_curve = pid_(&pid_curve, cmd_curve, sensor_curve);
		//pid_print(&pid_curve);

		float v = regul_sum / 2.0;//ratio_speed_sensor*(output->vitesse1_ratio + output->vitesse2_ratio) /2.0;
		output->vitesse1_ratio = v*(regul_curve+1);
		output->vitesse2_ratio = v*(1-regul_curve);
	} else {
		output->vitesse1_ratio *= ratio_speed_sensor;
		output->vitesse2_ratio *= ratio_speed_sensor;
	}

	float r = (sensor1 - sensor2) / (sensor1 + sensor2);
	printf(" r=%f (%f %.4f)\r\n", r, r_, (r_-r)/r_);
}
