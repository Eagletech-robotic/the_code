#pragma once
#include <stdint.h>

#include "robotic/pid.hpp"

enum class Color { BLUE = 0, YELLOW = 1 };
typedef enum getbleacher_state_t { GB_RESET, GB_NON_FREE, GB_CLOSE, GB_GET_IT } getbleacher_state_t;
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
    bool previous_jack_removed;
    // --- Géré par autopilot
    PID_t pid_diff;
    PID_t pid_sum;
    // -- Position de l'adversaire
    float opponent_x_m;
    float opponent_y_m;
    // -- fsm de gestion de l'approche d'un bleacher
    int getbleacher_state;
} state_t;

void print_state(state_t *state);
void state_init(state_t *);

void save_imu_to_field_transform(state_t &state, float x_field, float y_field, float theta_field);
void convert_from_imu_to_field(state_t &state, float &out_x, float &out_y, float &out_theta);
