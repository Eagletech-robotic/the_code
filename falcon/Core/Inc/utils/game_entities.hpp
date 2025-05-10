#ifndef BLEACHER_HPP
#define BLEACHER_HPP

#include <array>

#include "utils/constants.hpp"

constexpr int BLEACHER_INFLUENCE_SIZE = 400;

// GameEntity struct
struct GameEntity {
    float x = 0.0f;
    float y = 0.0f;
    float orientation = 0.0f; // in radians

    GameEntity() = default;

    GameEntity(float x_val, float y_val, float orientation_val) : x(x_val), y(y_val), orientation(orientation_val) {}
};

// Bleacher class
class Bleacher : public GameEntity {
  public:
    Bleacher() = default;

    Bleacher(float x_val, float y_val, float orientation_val) : GameEntity(x_val, y_val, orientation_val) {}

    const std::array<std::array<float, BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM>,
                     BLEACHER_INFLUENCE_SIZE / SQUARE_SIZE_CM> &
    potential_field();

  private:
    float potential_function(float dx, float dy);
};

#endif // BLEACHER_HPP
