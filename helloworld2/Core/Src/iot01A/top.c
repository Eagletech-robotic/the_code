/*
 * top.c
 *
 *  Created on: Sep 28, 2024
 *      Author: nboulay
 */

// top niveau du code, main.c est remplit de code généré cela permet de séparer
#include <iot01A/top_driver.h>
#include "main.h" //generated code
#include "iot01A/sensors.h"
#include <stdio.h>
#include "iot01A/encoder.h"
#include "iot01A/output.h"
#include "iot01A/input.h"
#include "iot01A/config.h"
#include "iot01A/top.h"
#include "iot01A/motor.h"

//TODO : to move to input or ouput
extern TIM_HandleTypeDef htim1; //
extern TIM_HandleTypeDef htim2; // PWM1 et PWM2
extern TIM_HandleTypeDef htim3; // encoder 2
extern TIM_HandleTypeDef htim5; // encoder 1
extern TIM_HandleTypeDef htim15; // pwm sur la led


extern UART_HandleTypeDef huart4; // uart du connector ARD
extern UART_HandleTypeDef huart1; // uart pour debug par usb

config_t config;
input_t input;
output_t output;

// code executé au reset
void top_init_driver(){
	  //startToF();
	  printf("\r\ntop Hello world !\r\n");
	  input_init(&input);
	  output_init(&output);
	  top_init(&config);
	  motorInit();
}

// code executer à la fréquence du timer systick
//  écriture des IO
	//  commande PWM des 2 moteurs
	//  commande servo si on les utilises
	//  commande ouput
//  lecture des IOs
	// encodeuse roues
	// lecture du jack
	// lecture du bouton bleu (sur 2 cycles pour faire un antirebond ?)
	// lecture ADC si besoin
 	// lecture gyroscope
	// " accelero
	// " boussole
// couche du dessous :
	//  test du jack de démarrage pour lancer un match : enregistre le t0 du match
	//  test du bouton bleue pour un autotest ?

void top_in_loop() {
	//printf("encoder œ%lu %lu\r\n", encoder_get_value(&htim5), encoder_get_value(&htim3));
	input_get(&input);
	//printf("\033[H"); // curseur en haut à gauche sur un shell
	//input_print(&input);
	top_step(&config, &input, &output );
	//output_print(&output);
	output_set(&output);
	fflush(stdout); // forcer la sortie pour mieux la lire
}

int old_tick=0;
int top_is_time_to_start() {
	int now = HAL_GetTick();
	int step_ms = config.time_step_ms;
	if (old_tick+step_ms == now) {
		old_tick = now;
		return 1;
	} else if ((old_tick+step_ms) < now)  {
		printf("!! %i %i\r\n",now, now - old_tick);
		old_tick = now;
		return 1;
	}

	return 0;
}
