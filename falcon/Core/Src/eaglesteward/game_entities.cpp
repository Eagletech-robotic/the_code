#include "eaglesteward/game_entities.hpp"

#include "utils/angles.hpp"
#include "utils/myprintf.hpp"
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

inline bool floatEqual(float a, float b) { return fabs(a - b) < 0.01; }

std::array<GameEntity, 2> Bleacher::waypoints() const {
    const float nx = std::cos(orientation);
    const float ny = std::sin(orientation);

    // The central bleacher should be pushed
    if (is_easy_central()) {
        return {{//
                 {x + BLEACHER_WAYPOINT_DISTANCE * nx, y + BLEACHER_WAYPOINT_DISTANCE * ny, orientation},
                 {-1.0, -1.0, 0.0}}};
    }

    return {{
        {x + BLEACHER_WAYPOINT_DISTANCE * nx, y + BLEACHER_WAYPOINT_DISTANCE * ny, orientation},
        {x - BLEACHER_WAYPOINT_DISTANCE * nx, y - BLEACHER_WAYPOINT_DISTANCE * ny, angle_normalize(M_PI + orientation)},
    }};
}

bool Bleacher::is_accessible_bleacher(const RobotColour colour) const {
    if (!initial_position)
        return false;

    // The bleachers on the side of the robot
    if ((colour == RobotColour::Blue && is_left_side()) || (colour == RobotColour::Yellow && is_right_side())) {
        return false;
    }

    if (is_next_to_backstage()) {
        // May block pami's path
        return false;
    }

    return true;
}

bool Bleacher::is_easy_central() const {
    return floatEqual(y, 0.950f) && (floatEqual(x, FIELD_WIDTH_M - 1.100f) || floatEqual(x, 1.100f));
}

bool Bleacher::is_easy_side(const RobotColour colour) const {
    return floatEqual(y, 0.250f) &&
           (colour == RobotColour::Blue ? floatEqual(x, FIELD_WIDTH_M - 0.775f) : floatEqual(x, 0.775f));
}

bool Bleacher::is_reserved(const RobotColour colour) const {
    return floatEqual(y, 1.725f) &&
           (colour == RobotColour::Blue ? floatEqual(x, FIELD_WIDTH_M - 0.825f) : floatEqual(x, 0.825f));
}

bool Bleacher::is_next_to_backstage() const { return y >= 1.0f; }

GameEntity BuildingArea::available_slot() const {
    if (type == Type::Small) {
        return {x, y, orientation};
    } else {
        const float from_center = static_cast<float>(first_available_slot - 1) * 0.15f + 0.05f;
        return {x + std::cos(orientation) * from_center, y + std::sin(orientation) * from_center, orientation};
    }
}

GameEntity BuildingArea::waypoint() const {
    auto [slot_x, slot_y, orientation_] = available_slot();
    return {slot_x + std::cos(orientation) * BUILDING_AREA_WAYPOINT_DISTANCE,
            slot_y + std::sin(orientation) * BUILDING_AREA_WAYPOINT_DISTANCE, orientation};
}

bool BuildingArea::is_starting() const { return floatEqual(y, 0.225f); }

float BuildingArea::span_x(bool occupied_space_only) const {
    return is_horizontal() ? get_length_span(occupied_space_only) : BUILDING_AREA_WIDTH;
}

float BuildingArea::span_y(bool occupied_space_only) const {
    return is_horizontal() ? BUILDING_AREA_WIDTH : get_length_span(occupied_space_only);
}

float BuildingArea::get_length_span(bool occupied_space_only) const {
    if (occupied_space_only) {
        return static_cast<float>(first_available_slot) * BUILDING_AREA_LENGTH_SMALL;
    }

    return (type == Type::Small) ? BUILDING_AREA_LENGTH_SMALL : BUILDING_AREA_LENGTH_LARGE;
}