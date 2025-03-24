/*
 * input.c
 *
 *  Created on: Nov 1, 2024
 *      Author: nboulay
 */


#include "iot01A/input.h"
#include"iot01A/encoder.h"
#include "iot01A/sensors.h"
#include <stdio.h>
#include <inttypes.h>
#include "iot01A/async_uart.h"

extern TIM_HandleTypeDef htim1; //
extern TIM_HandleTypeDef htim2; // PWM1 et PWM2
extern TIM_HandleTypeDef htim3; // encoder 1
extern TIM_HandleTypeDef htim5; // encoder 2
extern TIM_HandleTypeDef htim15; // pwm sur la led
extern TIM_HandleTypeDef htim16; // pwm16 servo
extern TIM_HandleTypeDef htim17; // pwm17 servo



void input_init(input_t * input) {
	startToF();
	encoder_init(&htim5);
	encoder_init(&htim3);
	async_uart_init();
}

int64_t raw[2];
int64_t old[2];

// un compteur compte en 32 bit unsigned et cela fait des mauvaises surprises
int32_t angle_get(int64_t new_, int64_t old, int64_t max) {
	int64_t r = new_ - old;
	//printf("   %ld\r\n",(int)r);
	// cas du passage par le zero
	if(r > (max/2)) {
		r = r - max;
	} else if (r < (-max/2)) {
		r = max + r;
	}
	return r;
}

int is_jack_done() {
	 GPIO_PinState pinState = HAL_GPIO_ReadPin(JACK_GPIO_Port, JACK_Pin);

	 return pinState == GPIO_PIN_SET;
}

void input_get(input_t *input) {
	old[0]=raw[0];
	old[1]=raw[1];
	raw[0]=encoder_get_value(&htim3);
	raw[1]=encoder_get_value(&htim5);
	int dist_mm;
	startToF();
	getDistance(&dist_mm);

	//printf("  : %ld %ld\r\n", (int)raw[0], (int)raw[1]);
	input->encoder1 = -angle_get(raw[0],old[0],65535);
	input->encoder2 = angle_get(raw[1],old[1],4294967295);
	input->tof_m =  dist_mm / 1000.0;
	input->is_jack_gone = is_jack_done();
}

void input_print(input_t *input) {
	//printf("in: %ld %ld %d %f...\r\n", (int32_t)input->encoder1, (int32_t)input->encoder2, input->is_jack_gone, input->tof_m);
	printf("%.1f\r\n", input->tof_m*100);
}
