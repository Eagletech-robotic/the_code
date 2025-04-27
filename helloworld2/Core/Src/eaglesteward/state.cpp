/*
 * state.c
 *
 *  Created on: Apr 6, 2025
 *      Author: nboulay
 */

#include "eaglesteward/state.hpp"

#include "robotic/angle.hpp"
#include "utils/myprintf.hpp"

#include <math.h>

void print_state(state_t *state) {
    myprintf("S %.2f %.2f  %.1f  %.2f\n", state->x_m, state->y_m, state->theta_deg, state->filtered_tof_m);
}

void state_init(state_t *state) {
    state->elapsed_time_s = .0f;
    state->filtered_tof_m = .0f;
    state->previous_is_jack_gone = 0;
    state->start_time_ms = 0;
    state->target = 0;
    state->theta_deg = .0f;
    state->x_m = .0f;
    state->y_m = .0f;
}

/**
 * Converts coordinates from IMU coordinate system to field coordinate system
 */
void convert_from_imu_to_field(state_t &state, float &out_x, float &out_y, float &out_theta) {
    // Apply rotation and translation to convert coordinates
    float theta_offset_rad = state.theta_offset_deg * (M_PI / 180.0f);
    out_x = state.x_m * cos(theta_offset_rad) - state.y_m * sin(theta_offset_rad) + state.x_offset_m;
    out_y = state.x_m * sin(theta_offset_rad) + state.y_m * cos(theta_offset_rad) + state.y_offset_m;

    // Convert angle from IMU system to field system
    out_theta = state.theta_deg + state.theta_offset_deg;
    out_theta = angle_normalize_deg(out_theta);
}