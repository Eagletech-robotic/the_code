#include "utils/game_entities.hpp"

#include <cmath>
#include <stdexcept>

const std::array<std::array<float, BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM>, BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM>
    &Bleacher::potential_field() {
    constexpr int size = BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM;
    static std::array<std::array<float, size>, size> field;

    constexpr float center_x = size / 2.0f;
    constexpr float center_y = size / 2.0f;

    for (int loop_x = 0; loop_x < size; loop_x++) {
        for (int loop_y = 0; loop_y < size; loop_y++) {
            float dx = std::abs(static_cast<float>(loop_x) - center_x);
            float dy = std::abs(static_cast<float>(loop_y) - center_y);
            field[loop_x][loop_y] = potential_function(dx, dy);
        }
    }

    return field;
}

float Bleacher::potential_function(const float dx, const float dy) {
    constexpr int scale = BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM;
    float const clamped_dx = dx / static_cast<float>(scale) * static_cast<float>(M_PI);
    float const clamped_dy = dy / static_cast<float>(scale) * static_cast<float>(M_PI);
    float const value = -std::exp(-clamped_dx - clamped_dy) / (1 + clamped_dx * clamped_dx + clamped_dy * clamped_dy);
    return value * static_cast<float>(scale) / static_cast<float>(M_PI);
}