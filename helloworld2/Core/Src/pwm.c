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
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

void PWMset_1(TIM_HandleTypeDef *htim, float ratio) {
	uint32_t period = htim->Init.Period;
    htim->Instance->CCR1 = ratio* (1.0f*period);
	//printf("CCR1 : %li %f\r\n", htim->Instance->CCR1, ratio);
}

// 0.0<=ratio<=1.0
// cela permet de changer la fréquence et le compteur sans changer les utilisations
// PA6 timer 16 channel 1
// PA7 timer 17 channel 1
void PWMset( float ratio1, float ratio2) {
	PWMset_1(&htim16,ratio1);
	PWMset_1(&htim17,ratio2);
}

void PWMstart() {
	HAL_TIM_PWM_Start(&htim16, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim17, TIM_CHANNEL_1);
}

