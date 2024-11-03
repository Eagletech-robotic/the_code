#pragma once
/*
 * input.h
 *
 *  Created on: Oct 26, 2024
 *      Author: nboulay
 */
#include <stdint.h>

typedef struct input_t {
	int is_jack_gone;
	float tof;
	float gyro[3];
	float accelero[3];
	float compass[3];
	int last_wifi_data[10]; // pour de futur donnée par caméra
	int32_t encoder1;
	int32_t encoder2;
} input_t;

// fonction inutilisable dans le module eaglesteward
void input_init(input_t *input);
void input_get(input_t *input);
