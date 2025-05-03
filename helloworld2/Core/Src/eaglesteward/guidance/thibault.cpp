// AI for a robot playing coupe de france de robotique. We are not allowed any runtime memory allocation.

#include "eaglesteward/guidance/thibault.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "eaglesteward/motor.hpp"
#include "eaglesteward/robot_constants.hpp"
#include "eaglesteward/state.hpp"
#include "robotic/angle.hpp"
#include "robotic/command.hpp"
#include "robotic/eagle_packet.hpp"
#include "utils/constants.hpp"
#include "utils/game_entities.hpp"
#include "utils/myprintf.hpp"
#include "utils/sized_array.hpp"

state_t thibault_state;

float potential_field[FIELD_WIDTH_SQ][FIELD_HEIGHT_SQ]{};
SizedArray<Bleacher, 10> bleachers;

void add_walls() {
    constexpr float INFLUENCE_DISTANCE = 0.35f;
    constexpr float MAX_POTENTIAL = 35.0f;

    constexpr int wall_influence_squares = static_cast<int>(INFLUENCE_DISTANCE / SQUARE_SIZE_M);
    for (int i = 0; i < FIELD_WIDTH_SQ; i++) {
        for (int j = 0; j < FIELD_HEIGHT_SQ; j++) {
            if (i < wall_influence_squares) {
                potential_field[i][j] +=
                    MAX_POTENTIAL * static_cast<float>(wall_influence_squares - i) / wall_influence_squares;
            } else if (i >= FIELD_WIDTH_SQ - wall_influence_squares) {
                potential_field[i][j] += MAX_POTENTIAL *
                                         static_cast<float>(i - (FIELD_WIDTH_SQ - wall_influence_squares)) /
                                         wall_influence_squares;
            }

            if (j < wall_influence_squares) {
                potential_field[i][j] +=
                    MAX_POTENTIAL * static_cast<float>(wall_influence_squares - j) / wall_influence_squares;
            } else if (j >= FIELD_HEIGHT_SQ - wall_influence_squares) {
                potential_field[i][j] += MAX_POTENTIAL *
                                         static_cast<float>(j - (FIELD_HEIGHT_SQ - wall_influence_squares)) /
                                         wall_influence_squares;
            }
        }
    }
}

void add_bleachers() {
    bleachers = {
        Bleacher(2.925f, 1.255f, 0.0f),
    };

    for (auto &bleacher : bleachers) {
        auto &field = bleacher.potential_field();

        int const field_width = static_cast<int>(field.size());
        int const field_height = static_cast<int>(field[0].size());

        int const bleacher_i = static_cast<int>(std::round(bleacher.x / SQUARE_SIZE_M)) - field_width / 2;
        int const bleacher_j = static_cast<int>(std::round(bleacher.y / SQUARE_SIZE_M)) - field_height / 2;

        for (int field_i = 0; field_i < field_width; field_i++) {
            for (int field_j = 0; field_j < field_height; field_j++) {
                int const i = bleacher_i + field_i;
                int const j = bleacher_j + field_j;

                if (i < 0 || j < 0 || i >= FIELD_WIDTH_SQ || j >= FIELD_HEIGHT_SQ)
                    continue;

                potential_field[i][j] += field[field_i][field_j];
            }
        }
    }
}

void init_potential_field() {
    add_walls();
    add_bleachers();
}

std::pair<Bleacher, float> get_closest_bleacher(float const x, float const y) {
    Bleacher *closest = &bleachers[0];
    float closest_distance = 9.999f;
    for (const auto bleacher : bleachers) {
        float const delta_x = std::abs(x - bleacher.x);
        float const delta_y = std::abs(y - bleacher.y);
        float const distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        if (distance < closest_distance)
            closest_distance = distance, *closest = bleacher;
    }

    return {*closest, closest_distance};
}

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

void next_command(state_t &state, const input_t &input, Command &command) {
    if (!input.jack_removed) {
        command.specialCommand = SpecialCommand::IMMEDIATE_STOP;
        myprintf("STOPPING because jack has not been removed\n");
        return;
    }

    float x, y, orientation_deg;
    get_position_and_orientation(state, x, y, orientation_deg);

    int const i = static_cast<int>(std::floor(x / SQUARE_SIZE_M));
    int const j = static_cast<int>(std::floor(y / SQUARE_SIZE_M));

    if (i >= FIELD_WIDTH_SQ || j >= FIELD_HEIGHT_SQ) {
        // throw std::out_of_range("Coordinates out of range");
    }

    myprintf("Position: x=%.3f y=%.3f angle=%.3f\n", x, y, orientation_deg);

    auto [closest_bleacher, closest_bleacher_distance] = get_closest_bleacher(x, y);
    myprintf("Closest target distance: %f\n", closest_bleacher_distance);
    myprintf("Target pos x: %f, y: %f\n", closest_bleacher.x, closest_bleacher.y);

    constexpr float STOP_DISTANCE = 0.25f;
    constexpr float MOVE_TO_TARGET_DISTANCE = 0.45f;
    if (closest_bleacher_distance <= STOP_DISTANCE) {
        myprintf("STOPPING because bleacher is near: %f\n", closest_bleacher_distance);
        command.specialCommand = SpecialCommand::IMMEDIATE_STOP;
        command.shovel = ShovelCommand::SHOVEL_EXTEND;
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

    float const dx = potential_field[i + LOOKAHEAD_DISTANCE][j] - potential_field[i - LOOKAHEAD_DISTANCE][j];
    float const dy = potential_field[i][j + LOOKAHEAD_DISTANCE] - potential_field[i][j - LOOKAHEAD_DISTANCE];

    if (std::abs(dx) / LOOKAHEAD_DISTANCE <= SLOPE_THRESHOLD && std::abs(dy) / LOOKAHEAD_DISTANCE <= SLOPE_THRESHOLD) {
        myprintf("STOPPING because slope is too flat - dx: %f, dy: %f\n", dx, dy);
        command.specialCommand = SpecialCommand::IMMEDIATE_STOP;
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
    printf("cycle : %.0f ms\r\n", config.time_step_s * 1000.0);
    motor_init(config, thibault_state);
    init_potential_field();
    state_init(thibault_state);
}

void thibault_top_step(const config_t &config, const input_t &input, output_t &output) {
    // 1. Debug: print input
    // print_complete_input(input);

    // 2. Update position and orientation from IMU and encoders
    update_state_from_input(config, input, thibault_state);

    // 3. Read the last Bluetooth packet (if available) and update the state
    update_state_from_bluetooth(thibault_state);

    // 4. Calculate the next command
    Command command{};
    next_command(thibault_state, input, command);

    // 5. Convert the command to actuator commands (output)
    set_output(config, input, command, output, thibault_state);

    // 6. Debug: print output
    // print_complete_output(output);
    // myprintf("Current potential: %f - Current orientation: %f\n", potential_field[i][j], orientation_deg);
}
