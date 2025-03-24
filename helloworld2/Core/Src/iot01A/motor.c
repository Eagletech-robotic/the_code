/*
 * motor.c
 *
 *  Created on: Aug 17, 2024
 *      Author: nboulay
 */

// Le but est d'ajouter le control de sens au control des PWM

#include "main.h"
#include "iot01A/pwm.h"

void motorInit() {
	PWMstart();
}

// -1<ratio<1, le bit de sens est modifié ici, puis le PWM mis à jour
// le sens ratio positif va vers l'avant.
void motorSet(float ratio1, float ratio2)
{
	if(ratio1 > 0.0f) {
		HAL_GPIO_WritePin(sens1_GPIO_Port, sens1_Pin, GPIO_PIN_SET);
	} else {
		ratio1 = -ratio1;
		HAL_GPIO_WritePin(sens1_GPIO_Port, sens1_Pin, GPIO_PIN_RESET);
	}

	if(ratio2 > 0.0f){
		HAL_GPIO_WritePin(sens2_GPIO_Port, sens2_Pin, GPIO_PIN_SET);
	} else {
		ratio2 = - ratio2;
		HAL_GPIO_WritePin(sens2_GPIO_Port, sens2_Pin, GPIO_PIN_RESET);
	}
	PWMset( ratio1, ratio2);
}
