#pragma once

#include "eaglesteward/state.hpp"
#include "iot01A//config.h"
#include "iot01A/input.h"
#include "iot01A/output.h"

enum class SpecialCommand { NONE, IMMEDIATE_STOP };
enum class ShovelCommand { SHOVEL_HOLD, SHOVEL_EXTEND, SHOVEL_RETRACT };
enum class LedCommand { LED_KEEP, LED_FLASH, LED_OFF };

struct Command {
    SpecialCommand specialCommand;
    float target_left_speed; // m/s, signed
    float target_right_speed;
    ShovelCommand shovel;
    LedCommand led;
};

void set_output(const config_t &config, const input_t &input, const Command &command, output_t &output, state_t &state);
