/*
 * state.c
 *
 *  Created on: Apr 6, 2025
 *      Author: nboulay
 */

#include "eaglesteward/state.h"

#include "utils/myprintf.h"

void print_state(state_t *state) {
    myprintf("S %.2f %.2f  %.1f  %.2f\n", state->x_m, state->y_m, state->theta_deg, state->filtered_tof_m);
}
