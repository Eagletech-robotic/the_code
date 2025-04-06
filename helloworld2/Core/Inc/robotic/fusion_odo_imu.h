/*
 * fusion_odo_imu.h
 *
 *  Created on: Apr 6, 2025
 *      Author: nboulay
 */
#pragma once
typedef struct fusion_odo_imu_t {
	float previous_float_yaw_imu_deg;
} fusion_odo_imu_t;

void fusion_odo_imu_init(fusion_odo_imu_t *fusion_odo_imu) ;
void fusion_odo_imu_fuse(
    struct fusion_odo_imu_t *fusion_odo_imu,
    float ax_imu_g, float ay_imu_g,  // non utilisés ici, mais disponibles si vous souhaitez pondérer
    float yaw_imu_deg,
    int delta_motor_left_ticks, int delta_motor_right_ticks,
    float dt_s,
    float theta_deg,
    float *delta_x_m, float *delta_y_m, float *delta_theta_deg,
    const float alpha_orientation_ratio,
    const float ticks_per_rev,
    const float wheel_circumference_m,
    const float wheel_base_m
);
