/*
 * state.c
 *
 *  Created on: Apr 6, 2025
 *      Author: nboulay
 */

#include "eaglesteward/state.h"
#include <stdio.h>

static int count = 0;
void print_state(state_t * state) {
	count ++;
	if(count == 250) {
		printf("S %.2f %.2f  %.1f\n", state->x_m, state->y_m, state->theta_deg);
		count = 0;
	}
}
