
// fonction de génération des sorties.
//  le but est de pouvoir tester les autres fonctions indépendement de la cible
//  et de générer les sorties de façon fixe temporrellement à chaque cycle

#include "iot01A/output.h"
#include "iot01A/motor.h"
#include <stdio.h>
#include "iot01A/pwm.h"
#include "robotic/myprintf.h"
#include "iot01A/led.h"
void output_init(output_t * output){
	output->vitesse1_ratio = 0.0f;
	output->vitesse2_ratio = 0.0f;
	output->servo_pelle_ratio = 0.0f; // 0.05 -> 1ms 0.1 -> 2ms, 0 off
	output->servo_en_plus = 0.0f;
}

void output_set(output_t *output) {
	if ( output->vitesse1_ratio < -1.0|| 1.0 < output->vitesse1_ratio) {
		printf("V1 survitesse (%f)\r\n",output->vitesse1_ratio);
	}
	if ( output->vitesse2_ratio < -1.0|| 1.0 < output->vitesse2_ratio) {
			printf("V2 survitesse (%f)\r\n",output->vitesse2_ratio);
	}
	motorSet(output->vitesse1_ratio, output->vitesse2_ratio);

	PWMSet_16(output->servo_pelle_ratio);
	PWMSet_17(output->servo_en_plus);
	led_1(output->led_ratio);
}

void output_print(output_t * output) {
	myprintf("O %.4f\t%.4f\t%.4f\n",output->vitesse1_ratio, output->vitesse2_ratio, output->servo_pelle_ratio);
	//myprintf("o: %.4f\t%.4f\r\n",output->vitesse1_ratio, output->vitesse2_ratio);
}
