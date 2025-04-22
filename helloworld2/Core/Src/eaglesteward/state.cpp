/*
 * state.c
 *
 *  Created on: Apr 6, 2025
 *      Author: nboulay
 */

#include "eaglesteward/state.hpp"

#include "utils/myprintf.hpp"

void print_state(state_t *state) {
    myprintf("S %.2f %.2f  %.1f  %.2f\n", state->x_m, state->y_m, state->theta_deg, state->filtered_tof_m);
}

void state_init(state_t * state ) {
	state->elapsed_time_s = .0f;
	state->filtered_tof_m = .0f;
	state->previous_is_jack_gone = 0;
	state->start_time_ms=0;
	state->target = 0;
	state->theta_deg = .0f;
	state->x_m = .0f;
	state->y_m = .0f;
}
