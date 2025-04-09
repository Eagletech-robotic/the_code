/*
 * controller_stanley.h
 *
 *  Created on: Apr 8, 2025
 *      Author: nboulay
 */
#pragma once


int stanley_controller(
    float robot_x_m, float robot_y_m, float robot_theta_deg,
    float x_start_m, float y_start_m,
    float x_target_m, float y_target_m,
    float x_next_m,   float y_next_m,
    float Vmax,
    float Wmax,
    float kStanley,
    float wheelBase_m,
    float arrivalThreshold,
    float *out_vitesse_droit,
    float *out_vitesse_gauche
);
