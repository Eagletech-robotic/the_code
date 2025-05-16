#include "eaglesteward/game_entities.hpp"

#include <cmath>

/**
 * Return the coordinates of the robot in the frame of the given entity.
 */
std::pair<float, float> GameEntity::position_in_local_frame(float robot_x, float robot_y) const {
    float const dx = robot_x - x;
    float const dy = robot_y - y;
    float const cos_o = std::cos(orientation);
    float const sin_o = std::sin(orientation);
    float const local_x = cos_o * dx + sin_o * dy;  // along orthogonal axis
    float const local_y = -sin_o * dx + cos_o * dy; // perpendicular axis
    return {local_x, local_y};
}

std::array<std::pair<float, float>, 2> Bleacher::waypoints() const {
    const float nx = std::cos(orientation);
    const float ny = std::sin(orientation);
    return {{
        {x + BLEACHER_WAYPOINT_DISTANCE * nx, y + BLEACHER_WAYPOINT_DISTANCE * ny},
        {x - BLEACHER_WAYPOINT_DISTANCE * nx, y - BLEACHER_WAYPOINT_DISTANCE * ny},
    }};
}

std::pair<float, float> BuildingArea::available_slot() const {
    if (type == Type::Small) {
        return {x, y};
    } else {
        const float from_center = static_cast<float>(first_available_slot - 1) * 0.15f;
        return {x + std::cos(orientation) * from_center, y + std::sin(orientation) * from_center};
    }
}

std::pair<float, float> BuildingArea::waypoint() const {
    auto [slot_x, slot_y] = available_slot();
    return {slot_x + std::cos(orientation) * BUILDING_AREA_WAYPOINT_DISTANCE,
            slot_y + std::sin(orientation) * BUILDING_AREA_WAYPOINT_DISTANCE};
}
