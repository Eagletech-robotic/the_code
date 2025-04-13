/*
 * state.c
 *
 *  Created on: Apr 6, 2025
 *      Author: nboulay
 */

#include "eaglesteward/state.h"
#include "robotic/myprintf.h"
void print_state(state_t * state) {
	myprintf("S %.2f %.2f  %.1f\n", state->x_m, state->y_m, state->theta_deg);
}
