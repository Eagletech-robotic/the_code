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

void move_to_target(Command &command, float const x, float const y, float const orientation_deg, float const target_x,
                    float const target_y) {
    float const delta_x = x - target_x;
    float const delta_y = y - target_y;
    float const target_angle_deg = std::atan2(-delta_y, -delta_x) / static_cast<float>(M_PI) * 180;
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
        command.target_left_speed = 0.5f - angle_diff / 180.0f;
        command.target_right_speed = 0.5f + angle_diff / 180.0f;
    }

    myprintf("Target angle: %f\n", target_angle_deg);
    myprintf("Angle diff: %f\n", angle_diff);
}

void next_command(const input_t &input, Command &command) {
    if (!input.jack_removed) {
        command.target_left_speed = 0.f;
        command.target_right_speed = 0.f;
        myprintf("STOPPING because jack has not been removed !!!\n");
        return;
    }

    auto const &world = thibault_state.world;

    float x, y, orientation_deg;
    thibault_state.getPositionAndOrientation(x, y, orientation_deg);

    int const i = static_cast<int>(std::floor(x / SQUARE_SIZE_M));
    int const j = static_cast<int>(std::floor(y / SQUARE_SIZE_M));

    myprintf("Position: x=%.3f y=%.3f angle=%.3f\n", x, y, orientation_deg);

    auto [closest_bleacher, closest_bleacher_distance] = world.closest_bleacher(x, y);
    myprintf("Closest target distance: %f\n", closest_bleacher_distance);
    myprintf("Target pos x: %f, y: %f\n", closest_bleacher.x, closest_bleacher.y);

    constexpr float STOP_DISTANCE = 0.25f;
    constexpr float MOVE_TO_TARGET_DISTANCE = 0.45f;
    if (closest_bleacher_distance <= STOP_DISTANCE) {
        myprintf("STOPPING because bleacher is near: %f\n", closest_bleacher_distance);
        command.target_left_speed = 0.f;
        command.target_right_speed = 0.f;
        command.shovel = ShovelCommand::SHOVEL_EXTENDED;
        return;
    }
    if (closest_bleacher_distance <= MOVE_TO_TARGET_DISTANCE) {
        myprintf("Moving to target");
        move_to_target(command, x, y, orientation_deg, closest_bleacher.x, closest_bleacher.y);
        return;
    }

    constexpr int LOOKAHEAD_DISTANCE = 5; // In squares
    constexpr float SLOPE_THRESHOLD = 0.05f;
    constexpr float MAX_SPEED = 1.0f; // m/s

    const auto &potential_field = world.potential_ready();
    float const dx = potential_field[i + LOOKAHEAD_DISTANCE][j] - potential_field[i - LOOKAHEAD_DISTANCE][j];
    float const dy = potential_field[i][j + LOOKAHEAD_DISTANCE] - potential_field[i][j - LOOKAHEAD_DISTANCE];

    if (std::abs(dx) / LOOKAHEAD_DISTANCE <= SLOPE_THRESHOLD && std::abs(dy) / LOOKAHEAD_DISTANCE <= SLOPE_THRESHOLD) {
        myprintf("STOPPING because slope is too flat - dx: %f, dy: %f\n", dx, dy);
        command.target_left_speed = 0.f;
        command.target_right_speed = 0.f;
    } else {
        float const target_angle_deg = std::atan2(-dy, -dx) / static_cast<float>(M_PI) * 180.0f;
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
            command.target_left_speed = MAX_SPEED / max * speed_left;
            command.target_right_speed = MAX_SPEED / max * speed_right;
        }

        myprintf("Target angle: %f\n", target_angle_deg);
        myprintf("Angle diff: %f\n", angle_diff);
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

    // 5. Convert the command to actuator commands (output)
    set_output(config, input, command, output, thibault_state);

    // 6. Debug: print output
    // print_complete_output(output);
    // myprintf("Current potential: %f - Current orientation: %f\n", potential_field[i][j], orientation_deg);
	myprintf("T %.2f (fin)\n", timer_get_us());

}
