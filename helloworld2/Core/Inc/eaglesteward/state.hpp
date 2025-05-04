#pragma once

#include "eaglesteward/world.hpp"
#include "iot01A/config.h"
#include "iot01A/input.h"

#include <stdint.h>

#include "robotic/pid.hpp"

enum class Color { BLUE = 0, YELLOW = 1 };
typedef enum getbleacher_state_t { GB_RESET, GB_NON_FREE, GB_CLOSE, GB_GET_IT } getbleacher_state_t;
typedef struct state_t {
    enum Color color;
    // IMU coordinate system
    float imu_x; // meters
    float imu_y;
    float imu_theta_deg;
    // IMU to field coordinate transformation
    float transformation_x;         // X translation offset after rotation (meters)
    float transformation_y;         // Y translation offset after rotation
    float transformation_theta_deg; // Rotation offset between IMU and field
    // Opponent
    float opponent_x; // meters
    float opponent_y;
    float opponent_theta_deg;
    // TOF
    float filtered_tof_m;
    // World
    World world;
    // --- Ecrit par retour
    uint32_t start_time_ms; // "date du début du match" en ms
    float elapsed_time_s;   // temps écoulé depuis le début du match
    bool previous_jack_removed;
    // --- Motor PID
    PID_t pid_diff;
    PID_t pid_sum;
    // -- fsm de gestion de l'approche d'un bleacher
    int target; // for rectangle test
    int getbleacher_state;
} state_t;

void print_state(const state_t &state);
void state_init(state_t &state);
void update_state_from_input(const config_t &config, const input_t &input, state_t &state);
void update_state_from_bluetooth(state_t &state);
void get_position_and_orientation(const state_t &state, float &out_x, float &out_y, float &out_theta);
