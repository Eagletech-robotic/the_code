// AI for a robot playing coupe de france de robotique, using gradient descent essentially. Since it
// is robotics, we are not allowed any memory allocation.

#include "eaglesteward/guidance/thibault.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "eaglesteward/motor.hpp"
#include "eaglesteward/pelle.hpp"
#include "eaglesteward/state.hpp"
#include "robotic/angle.h"
#include "robotic/constants.h"
#include "robotic/fusion_odo_imu.h"
#include "utils/constants.hpp"
#include "utils/game_entities.hpp"
#include "utils/myprintf.hpp"
#include "utils/sized_array.hpp"

state_t thibault_state;

float potential_field[P_FIELD_W][P_FIELD_H]{};
SizedArray<Bleacher, 10> bleachers;

void init_potential_field() {
    // -----------
    // Walls influence
    // -----------
    constexpr int wall_influence_size = 35 / SQUARE_SIZE_CM;
    constexpr int max_potential = 35;

    for (int x = 0; x < P_FIELD_W; x++) {
        for (int y = 0; y < P_FIELD_H; y++) {
            if (x < wall_influence_size) {
                potential_field[x][y] += static_cast<float>(max_potential * (wall_influence_size - x)) /
                                         static_cast<float>(wall_influence_size);
            } else if (x >= P_FIELD_W - wall_influence_size) {
                potential_field[x][y] += static_cast<float>(max_potential * (x - P_FIELD_W + wall_influence_size)) /
                                         static_cast<float>(wall_influence_size);
            }

            if (y < wall_influence_size) {
                potential_field[x][y] += static_cast<float>(max_potential * (wall_influence_size - y)) /
                                         static_cast<float>(wall_influence_size);
            } else if (y >= P_FIELD_H - wall_influence_size) {
                potential_field[x][y] += static_cast<float>(max_potential * (y - P_FIELD_H + wall_influence_size)) /
                                         static_cast<float>(wall_influence_size);
            }
        }
    }

    // -----------
    // Bleachers influence
    // -----------
    bleachers = {
        Bleacher(270, 70, 0),
    };

    for (auto &bleacher : bleachers) {
        auto &field = bleacher.potential_field();
        for (int x = 0; static_cast<decltype(field.size())>(x) < field.size(); x++) {
            for (int y = 0; static_cast<decltype(field.size())>(y) < field.size(); y++) {
                int const x_index = x + bleacher.x / SQUARE_SIZE_CM - static_cast<int>(field.size()) / 2;
                int const y_index = y + bleacher.y / SQUARE_SIZE_CM - static_cast<int>(field[0].size()) / 2;

                if (x_index >= P_FIELD_W || y_index >= P_FIELD_H || x_index < 0 || y_index < 0) {
                    continue;
                }

                potential_field[x_index][y_index] += field[x][y];
            }
        }
    }
}

void thibault_top_init(config_t *config) {
    config->time_step_ms = 4;
    printf("Cycle : %i ms\r\n", config->time_step_ms);
    autopilot_init(config, &thibault_state);
    init_potential_field();
}

std::pair<Bleacher, float> get_closest_bleacher(const float x_mm, const float y_mm) {
    Bleacher *closest = &bleachers[0];
    float closest_distance = 9999;
    for (const auto bleacher : bleachers) {
        const float delta_x = std::abs(x_mm - static_cast<float>(bleacher.x) * 10);
        const float delta_y = std::abs(y_mm - static_cast<float>(bleacher.y) * 10);
        const float distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        if (distance < closest_distance)
            closest_distance = distance, *closest = bleacher;
    }

    return {*closest, closest_distance};
}

void move_to_target(output_t *output, const float x_mm, const float y_mm, const float orientation_degrees,
                    const float target_x_mm, const float target_y_mm) {
    const float delta_x = x_mm - target_x_mm;
    const float delta_y = y_mm - target_y_mm;
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

void update_position_and_orientation(state_t *state, input_t *input, config_t *config) {
    float delta_x_m, delta_y_m, delta_theta_deg;
    fusion_odo_imu_fuse(input->imu_accel_x_mss, input->imu_accel_y_mss, input->delta_yaw_deg, input->encoder_left,
                        input->encoder_right, static_cast<float>(config->time_step_ms) / 1000.0f, state->theta_deg,
                        &delta_x_m, &delta_y_m, &delta_theta_deg, 0.5f, TICKS_PER_REV, WHEEL_CIRCUMFERENCE_M,
                        WHEELBASE_M);
    state->x_m += delta_x_m;
    state->y_m += delta_y_m;
    state->theta_deg += delta_theta_deg;
    state->theta_deg = angle_normalize_deg(state->theta_deg);
    print_state(state);
}

constexpr float INITIAL_ORIENTATION_DEGREES = 0.0f;
constexpr float INITIAL_X = 1.225f;
constexpr float INITIAL_Y = 1.775f;

void thibault_top_step(config_t *config, input_t *input, output_t *output) {
    if (!input->is_jack_gone) {
        output->motor_left_ratio = 0.0f;
        output->motor_right_ratio = 0.0f;
        myprintf("STOPPING because jack has not been removed\n");
        return;
    }

    update_position_and_orientation(&thibault_state, input, config);
    const float x_mm = INITIAL_X_MM - thibault_state.y_m * 1000;
    const float y_mm = INITIAL_Y_MM - thibault_state.x_m * 1000;
    const float orientation_degrees = INITIAL_ORIENTATION_DEGREES - thibault_state.theta_deg;

    int const index_x = std::floor(x_mm / 10.0f / SQUARE_SIZE_CM);
    int const index_y = std::floor(y_mm / 10.0f / SQUARE_SIZE_CM);

    if (index_x >= P_FIELD_W || index_y >= P_FIELD_H) {
        // throw std::out_of_range("Coordinates out of range");
    }

    myprintf("X %.3f %.3f %.3f\n", x_mm, y_mm, orientation_degrees);

    auto [closest_bleacher, closest_bleacher_distance] = get_closest_bleacher(x_mm, y_mm);

    if (closest_bleacher_distance <= STOP_DISTANCE_MM) {
        myprintf("STOPPING because bleacher is near: %f\n", closest_bleacher_distance);
        pelle_out(output);
        output->motor_left_ratio = 0.0f;
        output->motor_right_ratio = 0.0f;
        return;
    }
    if (closest_bleacher_distance < 45.0f) {
        move_to_target(output, x_mm, y_mm, orientation_degrees, static_cast<float>(closest_bleacher.x) * 10.0f,
                       static_cast<float>(closest_bleacher.y) * 10.0f);
        return;
    }

    constexpr int LOOKAHEAD_DISTANCE = 5;
    constexpr float SLOPE_THRESHOLD = 0.05f;
    const float dx =
        potential_field[index_x + LOOKAHEAD_DISTANCE][index_y] - potential_field[index_x - LOOKAHEAD_DISTANCE][index_y];
    const float dy =
        potential_field[index_x][index_y + LOOKAHEAD_DISTANCE] - potential_field[index_x][index_y - LOOKAHEAD_DISTANCE];

    if (std::abs(dx) / LOOKAHEAD_DISTANCE <= SLOPE_THRESHOLD && std::abs(dy) / LOOKAHEAD_DISTANCE <= SLOPE_THRESHOLD) {
        output->motor_left_ratio = 0.0f;
        output->motor_right_ratio = 0.0f;
        pelle_out(output);

        myprintf("STOPPING because slope is too flat - dx: %f, dy: %f\n", dx, dy);
    } else {
        const float target_angle_deg = std::atan2(-dx, dy) / static_cast<float>(M_PI) * 180.0f;

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

    myprintf("Left motor ratio: %f, Right motor ratio: %f\n", output->motor_left_ratio, output->motor_right_ratio);
    myprintf("Current orientation: %f\n", orientation_degrees);
    myprintf("Current potential: %f\n", potential_field[index_x][index_y]);
}
