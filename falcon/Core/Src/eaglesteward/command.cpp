#include "eaglesteward/command.hpp"

#include "eaglesteward/motor.hpp"

void set_output(const config_t &config, const input_t &input, const Command &command, output_t &output, State &state) {
    // Motors
    motor_calculate_ratios(config, state, input, command.target_left_speed, command.target_right_speed,
                           output.motor_left_ratio, output.motor_right_ratio);

    // Shovel
    switch (command.shovel) {
    case ShovelCommand::SHOVEL_EXTENDED:
        output.shovel_ratio = 0.105f;
        break;
    case ShovelCommand::SHOVEL_RETRACTED:
        output.shovel_ratio = 0.05f;
        break;
    }

    // LED
    if (command.led) {
        output.led_ratio = 0.5f;
    } else {
        output.led_ratio = 0.f;
    }
}
