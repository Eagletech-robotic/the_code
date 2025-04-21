// AI for a robot playing coupe de france de robotique, using gradient descent essentially. Since it
// is robotics, we are not allowed any memory allocation.

#include "eaglesteward/thibault.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>

#include "eaglesteward/pelle.h"
#include "eaglesteward/state.h"
#include "iot01A/top_driver.h"
#include "utils/constants.hpp"
#include "utils/debug.hpp"
#include "utils/game_entities.hpp"
#include "utils/myprintf.h"
#include "utils/sized_array.hpp"

float potential_field[P_FIELD_W][P_FIELD_H]{};

SizedArray<Bleacher, 10> bleachers;

void add_walls() {
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
}

extern "C" {

void thibault_top_init(config_t *config) {
    bleachers = {
        Bleacher(293, 67, 0), /*
Bleacher(82, 27, 0),  Bleacher(217, 27, 0),  Bleacher(222, 175, 0),
Bleacher(77, 175, 0),  Bleacher(190, 105, 0)},
Bleacher(292, 67, 0),  Bleacher(292, 160, 0),
Bleacher(7, 160, 0),*/
    };

    add_walls();

    for (auto &bleacher : bleachers) {
        auto &field = bleacher.potential_field();
        for (int x = 0; x < field.size(); x++) {
            for (int y = 0; y < field.size(); y++) {
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

} // extern "C"

std::pair<Bleacher, float> get_closest_bleacher(const float x_mm, const float y_mm) {
    Bleacher *closest = &bleachers[0];
    float closest_distance = 9999;
    for (const auto bleacher : bleachers) {
        float delta_x = std::abs(x_mm - bleacher.x * 10);
        float delta_y = std::abs(y_mm - bleacher.y * 10);
        float distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        if (distance < closest_distance)
            closest_distance = distance, *closest = bleacher;
    }

    return {*closest, closest_distance};
}

void move_to_target(const input_t *input, output_t *output, const float target_x_mm, const float target_y_mm) {
    const float delta_x = input->x_mm - target_x_mm;
    const float delta_y = input->y_mm - target_y_mm;
    auto target_angle_deg = std::atan2(delta_x, delta_y) / M_PI * 180;
    if (target_angle_deg < 0)
        target_angle_deg += 180;

    auto angle_diff = static_cast<float>(std::fmod(target_angle_deg - input->orientation_degrees, 360));
    if (angle_diff >= 180)
        angle_diff -= 360;

    if (std::abs(angle_diff) >= 90) {
        if (angle_diff <= 0) {
            output->motor_left_ratio = 0.0f;
            output->motor_right_ratio = 0.5f;
        } else {
            output->motor_left_ratio = 0.5f;
            output->motor_right_ratio = 0.0f;
        }
    } else {
        output->motor_left_ratio = 0.5f + angle_diff / 180.0f;
        output->motor_right_ratio = 0.5f - angle_diff / 180.0f;
    }
}

extern "C" {

/*void thibault_top_step_bridge(input_t* input, const state_t* state, output_t* output) {
    input->x_mm = -state->y_m * 1000 + 1225;
    input->y_mm = -state->x_m * 1000 + 1775;
    input->orientation_degrees = -state->theta_deg;
    thibault_top_step(input, state, output);
}*/

void thibault_top_step(config_t *config, input_t *input, output_t *output) {
    int const index_x = std::floor((input->x_mm / 10.0f) / SQUARE_SIZE_CM);
    int const index_y = std::floor((input->y_mm / 10.0f) / SQUARE_SIZE_CM);

    if (index_x >= P_FIELD_W || index_y >= P_FIELD_H) {
        // throw std::out_of_range("Coordinates out of range");
    }

    constexpr float VITESSE_RATIO_MAX = 1.2f;
    constexpr float STOP_DISTANCE = 275;

    myprintf("X %.3f %.3f %.3f\n", input->x_mm, input->y_mm, input->orientation_degrees);

    std::pair<Bleacher, float> closest_result = get_closest_bleacher(input->x_mm, input->y_mm);
    const Bleacher &closest_bleacher = closest_result.first;
    const float closest_bleacher_distance = closest_result.second;

    if (closest_bleacher_distance <= STOP_DISTANCE) {
        myprintf("STOPPING because bleacher is near: %f\n", closest_bleacher_distance);
        pelle_out(output);
        output->motor_left_ratio = 0.0f;
        output->motor_right_ratio = 0.0f;
        return;
    }
    if (closest_bleacher_distance < 45.0f) {
        move_to_target(input, output, closest_bleacher.x * 10, closest_bleacher.y * 10);
        return;
    }

    const int LOOKAHEAD_DISTANCE = 5;
    const float SLOPE_THRESHOLD = 0.05f;
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
        const float target_angle_deg = std::atan2(-dx, dy) / M_PI * 180.0f;

        auto angle_diff = target_angle_deg - input->orientation_degrees;
        if (angle_diff <= -180)
            angle_diff += 360;
        else if (angle_diff >= 180)
            angle_diff -= 360;

        if (std::abs(angle_diff) >= 90) {
            if (angle_diff <= 0) {
                output->motor_left_ratio = 0.0f;
                output->motor_right_ratio = 0.5f;
            } else {
                output->motor_left_ratio = 0.5f;
                output->motor_right_ratio = 0.0f;
            }
        } else {
            const float right = 0.5f - angle_diff / 180.0f;
            const float left = 0.5f + angle_diff / 180.0f;
            const float max = std::max(right, left);
            output->motor_left_ratio = VITESSE_RATIO_MAX / max * left;
            output->motor_right_ratio = VITESSE_RATIO_MAX / max * right;
        }

        myprintf("Target angle: %f\n", target_angle_deg);
        myprintf("Angle diff: %f\n", angle_diff);
    }

    myprintf("Left motor ratio: %f, Right motor ratio: %f\n", output->motor_left_ratio, output->motor_right_ratio);
    myprintf("Current orientation: %f\n", input->orientation_degrees);
    myprintf("Current potential: %f\n", potential_field[index_x][index_y]);
}

} // extern "C"
