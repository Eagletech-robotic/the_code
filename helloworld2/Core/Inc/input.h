#pragma once
/*
 * input.h
 *
 *  Created on: Oct 26, 2024
 *      Author: nboulay
 */


typedef struct input_t {
	int is_jack_gone;
	float tof;
	float gyro[3];
	float accelero[3];
	float compass[3];
	int last_wifi_data[10]; // pour de futur donnée par caméra
} input_t;
