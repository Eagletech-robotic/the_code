#pragma once

#include <stdint.h>

// !!!!!! IMPORTANT !!!!!!
// Changes to this structure must be reflected in the simulator.
typedef struct input_t {
    bool jack_removed;
    float tof_m;
    float delta_yaw; // différence avec l'appel précedent de ins.yaw
    int32_t delta_encoder_left;
    int32_t delta_encoder_right;
    float imu_yaw;
    float imu_accel_x_mss; // vers l'avant
    float imu_accel_y_mss; // vers la gauche
    float imu_accel_z_mss; // vers le haut
    int blue_button;
    uint32_t clock_ms; // free running counter in ms
} input_t;

// fonction inutilisable dans le module eaglesteward
void input_init(input_t &input);
void input_get(input_t &input);
void print_input(const input_t &input);
