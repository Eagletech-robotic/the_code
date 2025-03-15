// bleacher.hpp

#ifndef BLEACHER_HPP
#define BLEACHER_HPP

#include <array>
#include <cstdint>

#include "utils/constants.h"

constexpr int BLEACHER_INFLUENCE_SIZE = 150;

// GameEntity struct
struct GameEntity {
    int x;
    int y;
    int orientation_degrees;

    GameEntity() = default;

    GameEntity(int x, int y, int orientation_degrees);
};

// Bleacher class
class Bleacher : public GameEntity {
   public:
    const std::array<std::array<float, BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM>,
                     BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM>&
    potential_field();

   private:
    float potential_function(float dx, float dy);
};

#endif  // BLEACHER_HPP
