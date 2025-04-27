#pragma once
#include <stdint.h>

#include "robotic/pid.hpp"

enum class Color { BLUE = 0, YELLOW = 1 };

typedef struct state_t {
    enum Color color;
    // IMU coordinate system
    float x_m;
    float y_m;
    float theta_deg;
    // IMU to field coordinate transformation
    float theta_offset_deg; // Rotation offset between IMU and field
    float x_offset_m;       // X translation offset after rotation
    float y_offset_m;       // Y translation offset after rotation
    // ...
    int target; // for rectangle test
    float filtered_tof_m;
    // --- Ecrit par retour
    uint32_t start_time_ms; // "date du début du match" en ms
    float elapsed_time_s;   // temps écoulé depuis le début du match
    int previous_is_jack_gone;
    // --- Géré par autopilot
    PID_t pid_diff;
    PID_t pid_sum;
    // -- Position de l'adversaire
    float opponent_x_m;
    float opponent_y_m;
    // -- target en cours, cela peut être n'importe quoi
    float target_x_m;
    float target_y_m;
} state_t;

void print_state(state_t *state);
void state_init(state_t *);

void convert_from_imu_to_field(state_t &state, float &out_x, float &out_y, float &out_theta);
