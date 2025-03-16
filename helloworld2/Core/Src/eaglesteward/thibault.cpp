// AI for a robot playing coupe de france de robotique, using gradient descent essentially. Since it
// is robotics, we are not allowed any memory allocation.

#include "thibault.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

#include "iot01A/top_driver.h"
#include "utils/constants.hpp"
#include "utils/debug.hpp"
#include "utils/game_entities.hpp"
#include "utils/sized_array.hpp"

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
        Bleacher{{82, 27, 0}},  Bleacher{{217, 27, 0}},  Bleacher{{222, 175, 0}},
        Bleacher{{77, 175, 0}}, Bleacher{{110, 105, 0}}, Bleacher{{190, 105, 0}},
        Bleacher{{7, 67, 0}},   Bleacher{{292, 67, 0}},  Bleacher{{292, 160, 0}},
        Bleacher{{7, 160, 0}},
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

void thibault_top_step(config_t* config, input_t* input, output_t* output) {
    int const index_x = std::floor((input->x_mm / 10.0f) / SQUARE_SIZE_CM);
    int const index_y = std::floor((input->y_mm / 10.0f) / SQUARE_SIZE_CM);

    if (index_x >= P_FIELD_W || index_y >= P_FIELD_H) {
        throw std::out_of_range("Coordinates out of range");
    }

    float potentials[8] = {
        potential_field[index_x][index_y - 1], potential_field[index_x + 1][index_y - 1],
        potential_field[index_x + 1][index_y], potential_field[index_x + 1][index_y + 1],
        potential_field[index_x][index_y + 1], potential_field[index_x - 1][index_y + 1],
        potential_field[index_x - 1][index_y], potential_field[index_x - 1][index_y - 1],
    };

    std::pair<int, float> best_potential = std::make_pair(0, potentials[0]);
    for (int i = 0; i < 8; i++) {
        printf("Potential %d: %f\n", i, potentials[i]);
        if (potentials[i] < best_potential.second) {
            best_potential = {i, potentials[i]};
        }
    }

    if (best_potential.second > potential_field[index_x][index_y]) {
        output->vitesse1_ratio = 0;
        output->vitesse2_ratio = 0;
    } else {
        auto const target_angle_deg = static_cast<float>(best_potential.first * 45);

        auto angle_diff =
            static_cast<float>(std::fmod(target_angle_deg - input->orientation_degrees, 360));
        if (angle_diff < 0) angle_diff += 360;

        if (angle_diff <= 180) {
            output->vitesse1_ratio = (0.5f + angle_diff / 360) * 1.5f;
            output->vitesse1_ratio = std::min(output->vitesse1_ratio, 1.0f);
            output->vitesse2_ratio = 1 - output->vitesse1_ratio;
        } else {
            output->vitesse2_ratio = (0.5f + (360 - angle_diff) / 360) * 1.5f;
            output->vitesse2_ratio = std::min(output->vitesse2_ratio, 1.0f);
            output->vitesse1_ratio = 1 - output->vitesse2_ratio;
        }

        printf("Vitesse 1: %f, Vitesse 2: %f\n", output->vitesse1_ratio, output->vitesse2_ratio);
        printf("X: %d Y: %d\n", index_x, index_y);
        printf("Current potential: %f\n", potential_field[index_x][index_y]);
        printf("Best potential: %f\n", best_potential.second);
        printf("Target angle: %f\n", target_angle_deg);
        printf("Angle diff: %f\n", angle_diff);
    }
}
}

#ifdef STANDALONE
int main() {
    config_t config;

    thibault_top_init(&config);

    return 0;
}
#endif
