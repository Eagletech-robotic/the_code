#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include "eaglesteward/constants.hpp"
#include "robotic/eagle_packet.hpp"
#include "utils/sized_array.hpp"

class BuildingArea;

// GameEntity struct
struct GameEntity {
    float x = 0.0f;
    float y = 0.0f;
    float orientation = 0.0f; // in radians

    GameEntity() = default;

    GameEntity(float x_val, float y_val, float orientation_val) : x(x_val), y(y_val), orientation(orientation_val) {}

    bool operator==(const GameEntity &game_entity) const = default;

    [[nodiscard]] std::pair<float, float> position_in_local_frame(float robot_x, float robot_y) const;

    [[nodiscard]] bool is_horizontal() const { return std::abs(fmodf(orientation, M_PI)) < 0.1f; }
};

// Bleacher class
class Bleacher : public GameEntity {
  public:
    bool initial_position = false;

    Bleacher() = default;

    Bleacher(float x_val, float y_val, float orientation_val, bool initial_position_val)
        : GameEntity(x_val, y_val, orientation_val) {
        initial_position = initial_position_val;
    }

    [[nodiscard]] std::array<GameEntity, 2> waypoints() const;

    [[nodiscard]] bool is_accessible_bleacher(RobotColour colour) const;

    // The 2 bleachers in the centre
    [[nodiscard]] bool is_easy_central() const;

    // The bleacher closest to the starting position, on the side of our robot
    [[nodiscard]] bool is_easy_side(RobotColour colour) const;

    [[nodiscard]] bool is_left_side() const { return x < FIELD_WIDTH_M / 2.0; }

    [[nodiscard]] bool is_right_side() const { return x >= FIELD_WIDTH_M / 2.0; }

    // The reserved bleacher next to the backstage
    [[nodiscard]] bool is_reserved(RobotColour colour) const;

    // The 4 bleachers next to the backstage
    [[nodiscard]] bool is_next_to_backstage() const;
};

// BuildingArea class
class BuildingArea : public GameEntity {
  public:
    // x, y: position of the center of the building area

    enum class Type { Small, Large };

    Type type = Type::Small;
    RobotColour colour = RobotColour::Blue;
    int8_t first_available_slot = 0;

    BuildingArea() = default;

    BuildingArea(float x_val, float y_val, float orientation_val, Type type_val, RobotColour colour_val)
        : GameEntity(x_val, y_val, orientation_val) {
        type = type_val;
        colour = colour_val;
    }

    [[nodiscard]] GameEntity available_slot() const;
    [[nodiscard]] GameEntity waypoint() const;
    [[nodiscard]] uint8_t nb_slots() const { return type == Type::Small ? 1 : 2; }
    [[nodiscard]] bool is_full() const { return first_available_slot >= nb_slots(); }
    [[nodiscard]] bool is_starting() const;
    [[nodiscard]] float span_x(bool occupied_space_only) const;
    [[nodiscard]] float span_y(bool occupied_space_only) const;
    [[nodiscard]] float get_length_span(bool occupied_space_only) const;
};

// Backstage class
class Backstage : public GameEntity {
  public:
    RobotColour colour = RobotColour::Blue;

    Backstage() = default;

    Backstage(float x_val, float y_val, float orientation_val, RobotColour colour_val)
        : GameEntity(x_val, y_val, orientation_val) {
        colour = colour_val;
    }
};

// Can class
class Can : public GameEntity {
  public:
    Can() = default;

    Can(float x_val, float y_val, float orientation_val) : GameEntity(x_val, y_val, orientation_val) {}
};

// Plank class
class Plank : public GameEntity {
  public:
    Plank() = default;

    Plank(float x_val, float y_val, float orientation_val) : GameEntity(x_val, y_val, orientation_val) {}
};
