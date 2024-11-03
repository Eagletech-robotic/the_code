/*
 * input.c
 *
 *  Created on: Nov 1, 2024
 *      Author: nboulay
 */


#include "iot01A/input.h"
#include"iot01A/encoder.h"
#include "iot01A/sensors.h"

extern TIM_HandleTypeDef htim1; //
extern TIM_HandleTypeDef htim2; // PWM1 et PWM2
extern TIM_HandleTypeDef htim3; // encoder 2
extern TIM_HandleTypeDef htim5; // encoder 1
extern TIM_HandleTypeDef htim15; // pwm sur la led


void input_init(input_t * input) {
	startToF();
	encoder_init(&htim5);
	encoder_init(&htim3);
}

void input_get(input_t *input) {
	input->encoder1=encoder_get_value(&htim5);
	input->encoder2=encoder_get_value(&htim3);
}
