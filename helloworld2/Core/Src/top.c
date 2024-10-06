/*
 * top.c
 *
 *  Created on: Sep 28, 2024
 *      Author: nboulay
 */

// top niveau du code, main.c est remplit de code généré cela permet de séparer
#include "main.h" //generated code
#include "sensors.h"
#include <stdio.h>
#include "encoder.h"

extern TIM_HandleTypeDef htim1; // timer de gestion du temps (1khz) : TODO : utiliser systick c'est fait pour: HAL_GetTick()
extern TIM_HandleTypeDef htim3; // encoder 2
extern TIM_HandleTypeDef htim5; // encoder 1
extern TIM_HandleTypeDef htim15; // pwm sur la led
extern TIM_HandleTypeDef htim16; // PWM1
extern TIM_HandleTypeDef htim17; // PWM2

extern UART_HandleTypeDef huart4; // uart du connector ARD
extern UART_HandleTypeDef huart1; // uart pour debug par usb

// code executé au reset
void top_init(){
	  startToF();
	  printf("\r\nHello world !\r\n");
	  encoder_init(&htim3);
	  encoder_init(&htim5);
}

// code executer à la fréquence du timer htim1
//  écriture des IO
//  lecture des IOs
//  test du jack de démarrage pour lancer un match : enregistre le t0 du match
//  test du bouton bleue pour un autotest ?
void top_in_loop() {
	printf("encoder %lu %lu\r\n", encoder_get_value(&htim5), encoder_get_value(&htim3));

}

int old_tick=0;
int top_is_time_to_start() {
	int now = HAL_GetTick();
	int step_ms = 5;
	if (old_tick+step_ms == now) { // 1 ms
		old_tick = now;
		return 1;
	} else if ((old_tick+step_ms) < now)  {
		printf("!! %i %i\r\n",now, now - old_tick);
		old_tick = now;
		return 1;
	}
	return 0;
}
