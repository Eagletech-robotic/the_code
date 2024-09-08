/*
 * pwm.c
 *
 *  Created on: Aug 16, 2024
 *      Author: nboulay
 */
#include"main.h"
#include<stdio.h>

// Gestion des pwm pour le moteur
// Pour une fréquence de 50 khz, cela donne un compteur qui va jusqu'à 1600. (80 Mhz / 50 khz) sans prescaler
// tout cela est programmer dans le ioc

extern TIM_HandleTypeDef htim2;


void PWMset_(TIM_HandleTypeDef *htim, float ratio2, float ratio15) {
	uint32_t period = htim->Init.Period;

	htim->Instance->CCR1 = ratio15* (1.0f*period);
	htim->Instance->CCR3 = ratio2 * (1.0f*period);

	//printf("CCR1 : %li %f\r\n", htim->Instance->CCR1, ratio15);
}

// 0.0<=ratio<=1.0
// cela permet de changer la fréquence et le compteur sans changer les utilisations
// PA2 timer 2 channel 3
// PA15 timer 2 channel 1
void PWMset( float ratio2, float ratio15) {
	PWMset_(&htim2,ratio2,ratio15);
}

void PWMstart() {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}

