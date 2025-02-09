
// fonction de génération des sorties.
//  le but est de pouvoir tester les autres fonctions indépendement de la cible
//  et de générer les sorties de façon fixe temporrellement à chaque cycle

#include "iot01A/output.h"
#include "iot01A/motor.h"
#include <stdio.h>

void output_init(output_t * output){
	output->vitesse1_ratio = 0;
	output->vitesse2_ratio = 0;
}


void output_set(output_t *output) {
	if ( output->vitesse1_ratio < -1.0|| 1.0 < output->vitesse1_ratio) {
		printf("V1 survitesse (%f)\r\n",output->vitesse1_ratio);
	}
	if ( output->vitesse2_ratio < -1.0|| 1.0 < output->vitesse2_ratio) {
			printf("V2 survitesse (%f)\r\n",output->vitesse2_ratio);
	}
	motorSet(output->vitesse1_ratio, output->vitesse2_ratio);
}

void output_print(output_t * output) {
	//printf("out: %.4f\t%.4f\t%.4f\r\n",output->vitesse1_ratio, output->vitesse2_ratio, output->servo_pelle_ratio);
	printf("o: %.4f\t%.4f\r\n",output->vitesse1_ratio, output->vitesse2_ratio);
}
