#include "utils/game_entities.hpp"

#include <cmath>
#include <stdexcept>
/*
GameEntity::GameEntity(const int x, const int y, const int orientation_degrees) {
    // if (x >= 300 || y >= 200) {
    //     throw std::out_of_range("Coordinates out of range");
    // }
    this->x = x;
    this->y = y;
    this->orientation_degrees = orientation_degrees;
}
*/

const std::array<std::array<float, BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM>,
                 BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM>&
Bleacher::potential_field() {
    constexpr int size = BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM;
    static std::array<std::array<float, size>, size> field;

    constexpr float center_x = size / 2.0f;
    constexpr float center_y = size / 2.0f;

    for (int x = 0; x < size; x++) {
        for (int y = 0; y < size; y++) {
            float dx = std::abs(static_cast<float>(x) - center_x);
            float dy = std::abs(static_cast<float>(y) - center_y);
            field[x][y] = potential_function(dx, dy);
        }
    }

    return field;
}

float Bleacher::potential_function(const float dx, const float dy) {
    constexpr int scale = BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM;
    float const clamped_dx = dx / static_cast<float>(scale) * static_cast<float>(M_PI);
    float const clamped_dy = dy / static_cast<float>(scale) * static_cast<float>(M_PI);
    float const value = -std::exp(-clamped_dx - clamped_dy) /
                        (1 + clamped_dx * clamped_dx + clamped_dy * clamped_dy);
    return value * static_cast<float>(scale) / static_cast<float>(M_PI);
}