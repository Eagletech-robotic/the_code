// AI for a robot playing coupe de france de robotique. We are not allowed any runtime memory allocation.

#include "eaglesteward/guidance/thibault.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "eaglesteward/command.hpp"
#include "eaglesteward/motor.hpp"
#include "eaglesteward/robot_constants.hpp"
#include "eaglesteward/state.hpp"
#include "eaglesteward/world.hpp"
#include "iot01A/top_driver.h"
#include "robotic/angle.hpp"
#include "robotic/eagle_packet.hpp"
#include "utils/constants.hpp"
#include "utils/myprintf.hpp"

State thibault_state;

void next_command(const input_t &input, Command &command) {
    if (!input.jack_removed) {
        command.target_left_speed = 0.f;
        command.target_right_speed = 0.f;
        command.led = true;
        myprintf("STOPPING because jack has not been removed !!!\n");
        return;
    }

    auto &world = thibault_state.world;
    world.set_target(TargetType::BleacherWaypoint);

    float x, y, orientation_deg;
    thibault_state.getPositionAndOrientation(x, y, orientation_deg);
    myprintf("Position: x=%.3f y=%.3f angle=%.3f\n", x, y, orientation_deg);

    float speed, target_angle_deg;
    world.potential_field_descent(x, y, speed, target_angle_deg);

    if (speed == 0.f) {
        command.target_left_speed = 0.f;
        command.target_right_speed = 0.f;
    } else {
        float const angle_diff = angle_normalize_deg(target_angle_deg - orientation_deg);

        if (std::abs(angle_diff) >= 90) {
            if (angle_diff >= 0) {
                command.target_left_speed = 0.0f;
                command.target_right_speed = 0.5f;
            } else {
                command.target_left_speed = 0.5f;
                command.target_right_speed = 0.0f;
            }
        } else {
            float const speed_left = 0.5f - angle_diff / 180.0f;
            float const speed_right = 0.5f + angle_diff / 180.0f;
            float const max = std::max(speed_left, speed_right);
            command.target_left_speed = speed / max * speed_left;
            command.target_right_speed = speed / max * speed_right;
        }
    }
}

void thibault_top_init(config_t &config) {
    config.time_step_s = 0.004f;
    thibault_state.init();
    motor_init(config, thibault_state);
}

void thibault_top_step(const config_t &config, const input_t &input, output_t &output) {
    // 1. Debug: print input
    // print_complete_input(input);

    // 2. Read the last Bluetooth packet (if available) and update the state
    thibault_state.updateFromBluetooth();

    // 3. Update position and orientation from IMU and encoders
    thibault_state.updateFromInput(config, input);
    myprintf("T %.2f (input)\n", timer_get_us());

    // 4. Calculate the next command
    Command command{};
    next_command(input, command);
    myprintf("T %.2f (command)\n", timer_get_us());

    // 5. Perform some computations that fit in a step
    thibault_state.world.do_some_calculations();

    // 6. Convert the command to actuator commands (output)
    set_output(config, input, command, output, thibault_state);

    // 7. Debug: print output
    // print_complete_output(output);
    // myprintf("Current potential: %f - Current orientation: %f\n", potential_field[i][j], orientation_deg);
    myprintf("T %.2f (fin)\n", timer_get_us());
}
