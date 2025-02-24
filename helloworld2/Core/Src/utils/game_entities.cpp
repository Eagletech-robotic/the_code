#include "utils/game_entities.hpp"

#include <cmath>
#include <stdexcept>

GameEntity::GameEntity(uint16_t x, uint16_t y, uint16_t orientation_degrees) {
    if (x >= 300 || y >= 200) {
        throw std::out_of_range("Coordinates out of range");
    }
    this->x = x;
    this->y = y;
    this->orientation_degrees = orientation_degrees;
}

const std::array<std::array<float, BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM>,
                 BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM>&
Bleacher::potential_field() {
    constexpr int size = BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM;
    static std::array<std::array<float, size>, size> field;

    float center_x = size / 2.0f;
    float center_y = size / 2.0f;

    for (int x = 0; x < size; x++) {
        for (int y = 0; y < size; y++) {
            float dx = std::abs(x - center_x);
            float dy = std::abs(y - center_y);
            field[x][y] = potential_function(dx, dy);
        }
    }

    return field;
}

float Bleacher::potential_function(float dx, float dy) {
    int scale = BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM;
    float clamped_dx = dx / scale * M_PI;
    float clamped_dy = dy / scale * M_PI;
    float value = -std::exp(-clamped_dx - clamped_dy) /
                  (1 + clamped_dx * clamped_dx + clamped_dy * clamped_dy);
    return value * scale / M_PI;
}