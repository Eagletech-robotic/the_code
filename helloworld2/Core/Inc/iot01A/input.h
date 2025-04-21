#pragma once
/*
 * input.h
 *
 *  Created on: Oct 26, 2024
 *      Author: nboulay
 */
#include <stdint.h>
// #include "robotic/um7.h" //TODO à virer

typedef struct input_t {
    int is_jack_gone;
    float tof_m;
    float x_mm;                // obsolète
    float y_mm;                // obsolète
    float orientation_degrees; // obsolète
    float delta_yaw_deg;       // différence avec l'appel précedent de ins.yaw
    // float gyro[3];
    // float accelero[3];
    // float compass[3];
    int32_t encoder_left;
    int32_t encoder_right;
    // INS
    // BT
    // um7_t ins;
    float imu_yaw_deg;
    float imu_accel_x_mss; // vers l'avant
    float imu_accel_y_mss; // vers la gauche
    float imu_accel_z_mss; // vers le haut
    int blue_button;
} input_t;

// fonction inutilisable dans le module eaglesteward
void input_init(input_t *input);
void input_get(input_t *input);
void input_print(input_t *input);
