// AI for a robot playing coupe de france de robotique, using gradient descent essentially. Since it
// is robotics, we are not allowed any memory allocation.

#include "thibault.hpp"
#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "eaglesteward/state.h"
#include "iot01A/top_driver.h"
#include "utils/constants.hpp"
#include "utils/debug.hpp"
#include "utils/game_entities.hpp"
#include "utils/sized_array.hpp"
#include "robotic/myprintf.h"
#include "eaglesteward/pelle.h"

float potential_field[P_FIELD_W][P_FIELD_H]{};

SizedArray<Bleacher, 10> bleachers;

void add_walls() {
    constexpr int wall_influence_size = 35 / SQUARE_SIZE_CM;
    constexpr int max_potential = 35;

    for (int x = 0; x < P_FIELD_W; x++) {
        for (int y = 0; y < P_FIELD_H; y++) {
            if (x < wall_influence_size) {
                potential_field[x][y] +=
                    static_cast<float>(max_potential * (wall_influence_size - x)) /
                    static_cast<float>(wall_influence_size);
            } else if (x >= P_FIELD_W - wall_influence_size) {
                potential_field[x][y] +=
                    static_cast<float>(max_potential * (x - P_FIELD_W + wall_influence_size)) /
                    static_cast<float>(wall_influence_size);
            }

            if (y < wall_influence_size) {
                potential_field[x][y] +=
                    static_cast<float>(max_potential * (wall_influence_size - y)) /
                    static_cast<float>(wall_influence_size);
            } else if (y >= P_FIELD_H - wall_influence_size) {
                potential_field[x][y] +=
                    static_cast<float>(max_potential * (y - P_FIELD_H + wall_influence_size)) /
                    static_cast<float>(wall_influence_size);
            }
        }
    }
}

extern "C" {

void thibault_top_init(config_t* config) {
    bleachers = {
        Bleacher{{293, 67, 0}}, /*
Bleacher{{82, 27, 0}},  Bleacher{{217, 27, 0}},  Bleacher{{222, 175, 0}},
Bleacher{{77, 175, 0}},  Bleacher{{190, 105, 0}},
Bleacher{{292, 67, 0}},  Bleacher{{292, 160, 0}},
Bleacher{{7, 160, 0}},*/
    };

    add_walls();

    for (auto& bleacher : bleachers) {
        auto& field = bleacher.potential_field();
        for (int x = 0; x < field.size(); x++) {
            for (int y = 0; y < field.size(); y++) {
                int const x_index =
                    x + bleacher.x / SQUARE_SIZE_CM - static_cast<int>(field.size()) / 2;
                int const y_index =
                    y + bleacher.y / SQUARE_SIZE_CM - static_cast<int>(field[0].size()) / 2;

                if (x_index >= P_FIELD_W || y_index >= P_FIELD_H || x_index < 0 || y_index < 0) {
                    continue;
                }

                potential_field[x_index][y_index] += field[x][y];
            }
        }
    }

#ifdef STANDALONE
    visualize_potential_field(potential_field, P_FIELD_W, P_FIELD_H);
#endif
}

}  // extern "C"

std::pair<Bleacher, int> get_closest_bleacher(const float x_mm, const float y_mm) {
    Bleacher closest;
    float closest_distance = 9999;
    for (const auto bleacher : bleachers) {
        float delta_x = std::abs(x_mm - bleacher.x * 10);
        float delta_y = std::abs(y_mm - bleacher.y * 10);
        float distance = std::sqrt(delta_x * delta_x + delta_y * delta_y);
        if (distance < closest_distance) closest_distance = distance, closest = bleacher;
    }

    return {closest, closest_distance};
}

void move_to_target(const input_t* input, output_t* output, const float target_x_mm,
                    const float target_y_mm) {
    const float delta_x = input->x_mm - target_x_mm;
    const float delta_y = input->y_mm - target_y_mm;
    auto target_angle_deg = std::atan2(delta_x, delta_y) / M_PI * 180;
    if (target_angle_deg < 0) target_angle_deg += 180;

    auto angle_diff =
        static_cast<float>(std::fmod(target_angle_deg - input->orientation_degrees, 360));
    if (angle_diff >= 180) angle_diff -= 360;

    if (std::abs(angle_diff) >= 90) {
        if (angle_diff <= 0) {
            output->vitesse1_ratio = 0.5f;
            output->vitesse2_ratio = 0.0f;
        } else {
            output->vitesse1_ratio = 0.0f;
            output->vitesse2_ratio = 0.5f;
        }
    } else {
        output->vitesse1_ratio = 0.5f - angle_diff / 180.0f;
        output->vitesse2_ratio = 0.5f + angle_diff / 180.0f;
    }
}

extern "C" {

void thibault_top_step_bridge(input_t* input, const state_t* state, output_t* output) {
    input->x_mm = -state->y_m * 1000 + 1225;
    input->y_mm = -state->x_m * 1000 + 1775;
    input->orientation_degrees = - state->theta_deg;
    thibault_top_step(input, state, output);
}

void thibault_top_step(input_t* input, const state_t* state, output_t* output) {
    int const index_x = std::floor((input->x_mm / 10.0f) / SQUARE_SIZE_CM);
    int const index_y = std::floor((input->y_mm / 10.0f) / SQUARE_SIZE_CM);

    if (index_x >= P_FIELD_W || index_y >= P_FIELD_H) {
        throw std::out_of_range("Coordinates out of range");
    }

    myprintf("X %.3f %.3f %.3f\n", input->x_mm, input->y_mm, input->orientation_degrees);

    const int LOOKAHEAD_DISTANCE = 5;
    const float SLOPE_THRESHOLD = 0.35f;
    const float dx = potential_field[index_x + LOOKAHEAD_DISTANCE][index_y] -
                     potential_field[index_x - LOOKAHEAD_DISTANCE][index_y];
    const float dy = potential_field[index_x][index_y + LOOKAHEAD_DISTANCE] -
                     potential_field[index_x][index_y - LOOKAHEAD_DISTANCE];

    if (std::abs(dx) / LOOKAHEAD_DISTANCE <= SLOPE_THRESHOLD &&
        std::abs(dy) / LOOKAHEAD_DISTANCE <= SLOPE_THRESHOLD) {
        output->vitesse1_ratio = 0;
        output->vitesse2_ratio = 0;
        pelle_out(output);
    } else {
        const float target_angle_deg = std::atan2(-dx, dy) / M_PI * 180.0f;

        auto angle_diff = target_angle_deg - input->orientation_degrees;
        if (angle_diff <= -180)
            angle_diff += 360;
        else if (angle_diff >= 180)
            angle_diff -= 360;

        if (std::abs(angle_diff) >= 90) {
            if (angle_diff <= 0) {
                output->vitesse1_ratio = 0.6f;
                output->vitesse2_ratio = 0.0f;
            } else {
                output->vitesse1_ratio = 0.0f;
                output->vitesse2_ratio = 0.6f;
            }
        } else {
            output->vitesse1_ratio = 0.5f - angle_diff / 180.0f;
            output->vitesse2_ratio = 0.5f + angle_diff / 180.0f;
        }

        /*printf("Vitesse 1: %f, Vitesse 2: %f\n", output->vitesse1_ratio, output->vitesse2_ratio);
        printf("Current orientation: %f\n", input->orientation_degrees);
        printf("Current potential: %f\n", potential_field[index_x][index_y]);
        printf("Target angle: %f\n", target_angle_deg);
        printf("Angle diff: %f\n", angle_diff);*/
    }
}

}  // extern "C"

#ifdef STANDALONE
int main() {
    config_t config;

    thibault_top_init(&config);

    return 0;
}
#endif
