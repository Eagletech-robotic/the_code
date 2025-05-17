#pragma once

#include <array>
#include <cmath>
#include <cstdint>

#include "robotic/constants.hpp"
#include "robotic/eagle_packet.hpp"
#include "utils/sized_array.hpp"

class BuildingArea;

// GameEntity struct
struct GameEntity {
    float x = 0.0f;
    float y = 0.0f;
    float orientation = 0.0f; // in radians

    GameEntity() = default;

    GameEntity(float x_val, float y_val, float orientation_val) : x(x_val), y(y_val), orientation(orientation_val) {
    }

    bool operator==(const GameEntity &game_entity) const = default;

    [[nodiscard]] std::pair<float, float> position_in_local_frame(float robot_x, float robot_y) const;
};

// Bleacher class
class Bleacher : public GameEntity {
public:
    Bleacher() = default;

    Bleacher(float x_val, float y_val, float orientation_val) : GameEntity(x_val, y_val, orientation_val) {
    }

    [[nodiscard]] std::array<std::pair<float, float>, 2> waypoints() const;

    [[nodiscard]] bool in_building_area(const SizedArray<BuildingArea, 8> &building_areas) const;
};

// BuildingArea class
class BuildingArea : public GameEntity {
public:
    // x, y: position of the center of the building area

    enum class Type { Small, Large };

    Type type = Type::Small;
    RobotColour colour;
    int8_t first_available_slot = 0;

    BuildingArea() = default;

    BuildingArea(float x_val, float y_val, float orientation_val, Type type_val, RobotColour colour_val)
        : GameEntity(x_val, y_val, orientation_val) {
        type = type_val;
        colour = colour_val;
    }

    [[nodiscard]] std::pair<float, float> available_slot() const;

    [[nodiscard]] std::pair<float, float> waypoint() const;

    [[nodiscard]] uint8_t nb_available_slots() const { return type == Type::Small ? 1 : 3 - first_available_slot; }
    [[nodiscard]] bool is_full() const { return first_available_slot == nb_available_slots(); }

    [[nodiscard]] float span_x() const {
        return is_horizontal()
                   ? (type == Type::Small ? BUILDING_AREA_LENGTH_SMALL : BUILDING_AREA_LENGTH_LARGE)
                   : BUILDING_AREA_WIDTH;
    }

    [[nodiscard]] float span_y() const {
        return is_horizontal()
                   ? BUILDING_AREA_WIDTH
                   : (type == Type::Small ? BUILDING_AREA_LENGTH_SMALL : BUILDING_AREA_LENGTH_LARGE);
    }

private:
    [[nodiscard]] bool is_horizontal() const { return std::abs(fmodf(orientation, M_PI)) < 0.1f; }
};
