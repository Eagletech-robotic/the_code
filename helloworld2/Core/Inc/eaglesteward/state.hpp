#pragma once
#include <stdint.h>

#include "robotic/pid.hpp"

typedef struct state_t {
    float x_m;
    float y_m;
    float theta_deg;
    int target; // for rectangle test
    float filtered_tof_m;
    // --- Ecrit par retour
    uint32_t start_time_ms; // "date du début du match" en ms
    float elapsed_time_s; // temps écoulé depuis le début du match
    int previous_is_jack_gone;
    // --- Géré par autopilot
    PID_t pid_diff;
    PID_t pid_sum;
} state_t;

void print_state(state_t *state);
void state_init(state_t *);
