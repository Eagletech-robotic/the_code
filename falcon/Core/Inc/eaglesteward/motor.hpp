#pragma once

#include "eaglesteward/state.hpp"
#include "iot01A/config.h"
#include "iot01A/input.h"

void motor_init(const config_t &config, State &state);
void motor_calculate_ratios(const config_t &config, State &state, const input_t &input, float speed_left_m_s,
                            float speed_right_m_s, float &out_motor_left_ratio, float &out_motor_right_ratio);