/*
 * state.h
 *
 *  Created on: Apr 6, 2025
 *      Author: nboulay
 */

#pragma once

typedef struct state_t {
	float x_m;
	float y_m;
	float theta_deg;
	int target; // for rectangle test
	float filtered_tof_m;
} state_t;

void print_state(state_t * state);
