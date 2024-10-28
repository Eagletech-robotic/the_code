/*
 * led.c
 *
 *  Created on: Sep 22, 2024
 *      Author: nboulay
 */

// Led PB14 TIM15_CH1
// Led PA5 qui est aussi sur le connecteur
//

#include "main.h"
#include "iot01A/led.h"


extern TIM_HandleTypeDef htim15;

void led_init() {
	HAL_TIM_PWM_Start(&htim15, TIM_CHANNEL_1);
	led_1(0.0);
}

// 0<r< 1.0
void led_1(float r) {
	htim15.Instance->CCR1 = r*1000;
}
