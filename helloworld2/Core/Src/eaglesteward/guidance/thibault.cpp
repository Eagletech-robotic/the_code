// AI for a robot playing coupe de france de robotique. We are not allowed any runtime memory allocation.

#include "eaglesteward/guidance/thibault.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "eaglesteward/motor.hpp"
#include "eaglesteward/pelle.hpp"
#include "eaglesteward/robot_constants.hpp"
#include "eaglesteward/state.hpp"
#include "robotic/angle.hpp"
#include "robotic/fusion_odo_imu.hpp"
#include "utils/constants.hpp"
#include "utils/debug.hpp"
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
        Bleacher(1.5f, 1.0f, 0.0f),
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

void thibault_top_init(config_t *config) {
    config->time_step_s = 0.004f;
    printf("cycle : %.0f ms\r\n", config->time_step_s * 1000.0);
    motor_init(*config, thibault_state);
    init_potential_field();
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

void move_to_target(const config_t *config, const input_t *input, output_t *output, float const x, float const y,
                    float const orientation_degrees, float const target_x, float const target_y) {
    float const delta_x = x - target_x;
    float const delta_y = y - target_y;
    auto target_angle_deg = std::atan2(delta_x, delta_y) / M_PI * 180;
    if (target_angle_deg < 0)
        target_angle_deg += 180;

    auto angle_diff = static_cast<float>(std::fmod(target_angle_deg - orientation_degrees, 360));
    if (angle_diff >= 180)
        angle_diff -= 360;

    float speed_left, speed_right;
    if (std::abs(angle_diff) >= 90) {
        if (angle_diff <= 0) {
            speed_left = 0.0f;
            speed_right = 0.5f;
        } else {
            speed_left = 0.5f;
            speed_right = 0.0f;
        }
    } else {
        speed_left = 0.5f + angle_diff / 180.0f;
        speed_right = 0.5f - angle_diff / 180.0f;
    }
    motor_calculate_ratios(*config, thibault_state, *input, speed_left, speed_right, output->motor_left_ratio,
                           output->motor_right_ratio);
}

void update_position_and_orientation(const input_t *input, const config_t *config) {
    float delta_x_m, delta_y_m, delta_theta_deg;
    fusion_odo_imu_fuse(input->imu_accel_x_mss, input->imu_accel_y_mss, input->delta_yaw_deg, input->delta_encoder_left,
                        input->delta_encoder_right, config->time_step_s, thibault_state.theta_deg, &delta_x_m, &delta_y_m,
                        &delta_theta_deg, 0.5f, TICKS_PER_REV, WHEEL_CIRCUMFERENCE_M, WHEELBASE_M);
    thibault_state.x_m += delta_x_m;
    thibault_state.y_m += delta_y_m;
    thibault_state.theta_deg += delta_theta_deg;
    thibault_state.theta_deg = angle_normalize_deg(thibault_state.theta_deg);
    print_state(&thibault_state);
}

constexpr float INITIAL_ORIENTATION_DEGREES = 0.0f;
constexpr float INITIAL_X = 1.225f;
constexpr float INITIAL_Y = 1.775f;

void thibault_top_step(const config_t *config, const input_t *input, output_t *output) {
    print_complete_input(*input);
    if (!input->is_jack_gone) {
        output->motor_left_ratio = 0.0f;
        output->motor_right_ratio = 0.0f;
        myprintf("STOPPING because jack has not been removed\n");
        return;
    }

    update_position_and_orientation(input, config);
    float const x = INITIAL_X - thibault_state.y_m; // Using different coordinate system than the state
    float const y = INITIAL_Y - thibault_state.x_m; // Using different coordinate system than the state
    float const orientation_degrees = INITIAL_ORIENTATION_DEGREES - thibault_state.theta_deg;

    int const i = static_cast<int>(std::floor(x / SQUARE_SIZE_M));
    int const j = static_cast<int>(std::floor(y / SQUARE_SIZE_M));

    if (i >= FIELD_WIDTH_SQ || j >= FIELD_HEIGHT_SQ) {
        throw std::out_of_range("Coordinates out of range");
    }

    myprintf("Position: x=%.3f y=%.3f angle=%.0f\n", x, y, orientation_degrees);

    auto [closest_bleacher, closest_bleacher_distance] = get_closest_bleacher(x, y);

    constexpr float STOP_DISTANCE = 0.275f;
    constexpr float MOVE_TO_TARGET_DISTANCE = 0.45f;
    if (closest_bleacher_distance <= STOP_DISTANCE) {
        myprintf("STOPPING because bleacher is near: %f\n", closest_bleacher_distance);
        output->motor_left_ratio = 0.0f;
        output->motor_right_ratio = 0.0f;
        pelle_out(output);
        return;
    }
    if (closest_bleacher_distance <= MOVE_TO_TARGET_DISTANCE) {
        move_to_target(config, input, output, x, y, orientation_degrees, closest_bleacher.x, closest_bleacher.y);
        return;
    }

    constexpr int LOOKAHEAD_DISTANCE = 5; // In squares
    constexpr float SLOPE_THRESHOLD = 0.05f;
    constexpr float VITESSE_MAX = 1.2f; // m/s

    float const dx = potential_field[i + LOOKAHEAD_DISTANCE][j] - potential_field[i - LOOKAHEAD_DISTANCE][j];
    float const dy = potential_field[i][j + LOOKAHEAD_DISTANCE] - potential_field[i][j - LOOKAHEAD_DISTANCE];

    if (std::abs(dx) / LOOKAHEAD_DISTANCE <= SLOPE_THRESHOLD && std::abs(dy) / LOOKAHEAD_DISTANCE <= SLOPE_THRESHOLD) {
        myprintf("STOPPING because slope is too flat - dx: %f, dy: %f\n", dx, dy);
        output->motor_left_ratio = 0.0f;
        output->motor_right_ratio = 0.0f;
        pelle_out(output);
    } else {
        float const target_angle_deg = std::atan2(-dx, dy) / static_cast<float>(M_PI) * 180.0f;

        auto angle_diff = target_angle_deg - orientation_degrees;
        if (angle_diff <= -180)
            angle_diff += 360;
        else if (angle_diff >= 180)
            angle_diff -= 360;

        if (std::abs(angle_diff) >= 90) {
            if (angle_diff <= 0) {
                motor_calculate_ratios(*config, thibault_state, *input, 0.0f, 0.5f, output->motor_left_ratio,
                                       output->motor_right_ratio);
            } else {
                motor_calculate_ratios(*config, thibault_state, *input, 0.5f, 0.0f, output->motor_left_ratio,
                                       output->motor_right_ratio);
            }
        } else {
            float const speed_left = 0.5f + angle_diff / 180.0f;
            float const speed_right = 0.5f - angle_diff / 180.0f;
            float const max = std::max(speed_left, speed_right);
            float const throttled_speed_left = VITESSE_MAX / max * speed_left;
            float const throttled_speed_right = VITESSE_MAX / max * speed_right;
            motor_calculate_ratios(*config, thibault_state, *input, throttled_speed_left, throttled_speed_right,
                                   output->motor_left_ratio, output->motor_right_ratio);
        }

        myprintf("Target angle: %f\n", target_angle_deg);
        myprintf("Angle diff: %f\n", angle_diff);
    }

    print_complete_output(*output);
    myprintf("Current potential: %f - Current orientation: %f\n", potential_field[i][j], orientation_degrees);
}
