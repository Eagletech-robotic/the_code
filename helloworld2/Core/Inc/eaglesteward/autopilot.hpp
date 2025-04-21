#pragma once

#include "eaglesteward/state.hpp"
#include "iot01A/config.h"
#include "iot01A/input.h"

void autopilot_init(const config_t *config, state_t *state);
void autopilot_step(const config_t *config, state_t *state, const input_t *input, float v_gauche_m_s,
                    float v_droite_m_s, float *out_motor_left_ratio, float *out_motor_right_ratio);