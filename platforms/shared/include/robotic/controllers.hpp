#pragma once
#include "robotic/pid.hpp"

bool stanley_controller(float robot_x, float robot_y, float robot_theta, float x_start, float y_start, float x_target,
                        float y_target, float x_next, float y_next, float Vmax, float Wmax, float Varrival,
                        float kStanley, float wheelBase, float arrivalThreshold, float *out_speed_left,
                        float *out_speed_right);

bool pid_controller(float robot_x, float robot_y, float robot_theta, float x_target, float y_target, float Vmax,
                    float w_max, float r_max, float wheelBase, float arrivalThreshold, float *out_speed_left,
                    float *out_speed_right);

void controllers_pid_init(PID_t *pid_theta_, PID_t *pid_speed_);

void limit_vw(float *v, float *w, float w_max, float r_max /* m, mettre <0 pour désactiver */);
