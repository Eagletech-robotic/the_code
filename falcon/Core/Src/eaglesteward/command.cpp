#include "eaglesteward/command.hpp"

#ifdef SIMULATOR
#include <cstring> // memcpy
#endif

#include "eaglesteward/motor.hpp"

void set_output(const config_t &config, const input_t &input, const Command &command, output_t &output, State &state) {
    // Motors
    motor_calculate_ratios(config, state, input, command.target_left_speed, command.target_right_speed,
                           output.motor_left_ratio, output.motor_right_ratio);

    const float speed_min = 0.1f;
    state.isMoving = fabs(output.motor_left_ratio + output.motor_right_ratio) > speed_min;

    if (output.motor_left_ratio + output.motor_right_ratio < 0.0f) {
    	state.isMovingForward = false;
    } else {
    	state.isMovingForward = true;
    }

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

#ifdef SIMULATOR
    std::memcpy(output.potential_field.front().data(), state.world.potential_ready().front().data(),
                sizeof(output.potential_field));
#endif
}
