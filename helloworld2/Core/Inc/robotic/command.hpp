#pragma once

#include "eaglesteward/state.hpp"
#include "iot01A//config.h"
#include "iot01A/input.h"
#include "iot01A/output.h"

enum class ShovelCommand { SHOVEL_RETRACTED, SHOVEL_EXTENDED };

struct Command {
    float target_left_speed = 0.f; // m/s, signed
    float target_right_speed = 0.f;
    ShovelCommand shovel = ShovelCommand::SHOVEL_RETRACTED;
    bool led = false;
};

void set_output(const config_t &config, const input_t &input, const Command &command, output_t &output, State &state);
