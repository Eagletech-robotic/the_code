#include "robotic/command.hpp"

#include "eaglesteward/motor.hpp"

void set_output(const config_t &config, const input_t &input, const Command &command, output_t &output, State &state) {
    // Motors
    if (command.specialCommand == SpecialCommand::IMMEDIATE_STOP) {
        output.motor_left_ratio = 0.0f;
        output.motor_right_ratio = 0.0f;
    } else {
        motor_calculate_ratios(config, state, input, command.target_left_speed, command.target_right_speed,
                               output.motor_left_ratio, output.motor_right_ratio);
    }

    // Shovel
    switch (command.shovel) {
    case ShovelCommand::SHOVEL_HOLD:
        // Do nothing
        break;
    case ShovelCommand::SHOVEL_EXTEND:
        output.servo_pelle_ratio = 0.12f;
        break;
    case ShovelCommand::SHOVEL_RETRACT:
        output.servo_pelle_ratio = 0.05f;
        break;
    }

    // LED
}
