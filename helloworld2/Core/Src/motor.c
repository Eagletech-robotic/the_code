/*
 * motor.c
 *
 *  Created on: Aug 17, 2024
 *      Author: nboulay
 */

// Le but est d'ajouter le control de sens au control des PWM

#include "main.h"
#include "pwm.h"

void motorInit() {
	PWMstart();
}

// -1<ratio<1, le bit de sens est modifié ici, puis le PWM mis à jour
void motorSet(float ratio2, float ratio15)
{
	if(ratio2 > 0.0f) {
		HAL_GPIO_WritePin(ARD_D8_GPIO_Port, ARD_D8_Pin, 0);
	} else {
		ratio2 = -ratio2;
		HAL_GPIO_WritePin(ARD_D8_GPIO_Port, ARD_D8_Pin, 1);
	}

	if(ratio15 > 0.0f){
		HAL_GPIO_WritePin(ARD_D7_GPIO_Port, ARD_D7_Pin, 0);
	} else {
		ratio15 = - ratio15;
		HAL_GPIO_WritePin(ARD_D7_GPIO_Port, ARD_D7_Pin, 1);
	}
	PWMset( ratio2, ratio15);
}
