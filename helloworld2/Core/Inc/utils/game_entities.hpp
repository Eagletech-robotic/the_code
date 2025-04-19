#ifndef BLEACHER_HPP
#define BLEACHER_HPP

#include <array>

#include "utils/constants.hpp"

constexpr int BLEACHER_INFLUENCE_SIZE = 400;

// GameEntity struct
struct GameEntity {
    int x;
    int y;
    int orientation_degrees;

    GameEntity() = default;

    GameEntity(int x_val, int y_val, int orientation_degrees_val)
        : x(x_val), y(y_val), orientation_degrees(orientation_degrees_val) {}
};

// Bleacher class
class Bleacher : public GameEntity {
   public:
    Bleacher(int x_val, int y_val, int orientation_degrees_val)
        : GameEntity(x_val, y_val, orientation_degrees_val) {}

    Bleacher() = default;

    const std::array<std::array<float, BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM>,
                     BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM>&
    potential_field();

   private:
    float potential_function(float dx, float dy);
};

#endif  // BLEACHER_HPP
