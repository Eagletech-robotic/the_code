#pragma once
#include"robotic/pid.hpp"
bool stanley_controller(float robot_x, float robot_y, float robot_theta, float x_start, float y_start, float x_target,
                        float y_target, float x_next, float y_next, float Vmax, float Wmax, float Varrival,
                        float kStanley, float wheelBase, float arrivalThreshold, float *out_speed_left,
                        float *out_speed_right);

bool pid_controller(float robot_x, float robot_y, float robot_theta, float x_target, float y_target, float Vmax,
                    float wheelBase, float arrivalThreshold, float *out_speed_left, float *out_speed_right);
void controllers_pid_init(PID_t *pid_theta_, PID_t *pid_speed_);
