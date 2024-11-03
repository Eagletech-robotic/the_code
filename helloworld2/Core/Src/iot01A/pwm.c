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
// PWM1 : TIM2_CH1
// PWM2 : TIM2_CH3

extern TIM_HandleTypeDef htim2;

void PWMset_1(TIM_HandleTypeDef *htim, float ratio1, float ratio2) {
	uint32_t period = htim->Init.Period;
    htim->Instance->CCR1 = ratio1* (1.0f*period);
    htim->Instance->CCR3 = ratio2* (1.0f*period);
	printf("! %li %li\r\n", htim->Instance->CCR1, htim->Instance->CCR3);
}

// 0.0<=ratio<=1.0
// cela permet de changer la fréquence et le compteur sans changer les utilisations

void PWMset( float ratio1, float ratio2) {
	PWMset_1(&htim2,ratio1, ratio2);
}

void PWMstart() {
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
}

