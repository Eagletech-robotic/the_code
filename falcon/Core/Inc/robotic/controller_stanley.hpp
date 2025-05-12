#pragma once

bool stanley_controller(float robot_x_m, float robot_y_m, float robot_theta, float x_start_m, float y_start_m,
                        float x_target_m, float y_target_m, float x_next_m, float y_next_m, float Vmax, float Wmax,
                        float kStanley, float wheelBase_m, float arrivalThreshold, float *out_speed_left,
                        float *out_speed_right);

bool controller_pid(float robot_x_m, float robot_y_m, float robot_theta, float x_target_m, float y_target_m, float Vmax,
                    float wheelBase_m, float arrivalThreshold, float *out_speed_left, float *out_speed_right);
