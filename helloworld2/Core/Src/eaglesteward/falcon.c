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
pid_t pid;

void top_init(config_t* config) {
	printf(":: %i\r\n",config->time_step_ms);
	carre_init(&carre, config->time_step_ms / 1000.0);
	pid_init(&pid);
	pid_tune(&pid, 0.000132 , 0.00006, 0.000000165); // Ku=0.00022 Tu = 110 ms
	pid_limits(&pid, -1.0, 1.0);
}

// top_loop doit appeler la fonction et gérer les IOS
void top_step(config_t* config, input_t *input, output_t* output ) {
	carre_in_loop(&carre, output);

	output->vitesse1_ratio = 4000.0 ;
	output->vitesse2_ratio = .1 ;// droite dans le sens de la marche

	float sensor = input->encoder1 ;//- input->encoder2;œ
	float cmd = output->vitesse1_ratio ;//- output->vitesse2_ratio;

	pid_command(&pid,cmd);
	float regul = pid_compute(&pid,sensor);
//	printf("  : %f %f\r\n", cmd, regul);
	//if(regul > 0.1) {
		pid_print(&pid);
	//}
	output->vitesse1_ratio = regul ;
	//output->vitesse2_ratio = .5 ;
	if(cmd > 0) {
//		output->vitesse1_ratio = output->vitesse1_ratio - regul;
	} else {
	//	output->vitesse2_ratio = output->vitesse2_ratio - regul;
	}
}
