#pragma once

#include "robotic/pid.h"

typedef struct state_t {
    float x_m;
    float y_m;
    float theta_deg;
    int target; // for rectangle test
    float filtered_tof_m;
    PID_t pid_diff;
    PID_t pid_sum;
} state_t;

void print_state(state_t *state);
