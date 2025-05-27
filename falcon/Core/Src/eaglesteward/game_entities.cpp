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

bool Bleacher::in_building_area(const SizedArray<BuildingArea, 8> &building_areas) const {
    for (const auto &building_area : building_areas) {
        if (std::abs(x - building_area.x) < (building_area.span_x() + BLEACHER_WIDTH) / 2 &&
            std::abs(y - building_area.y) < (building_area.span_y() + BLEACHER_WIDTH) / 2) {
            return true;
        }
    }
    return false;
}

bool Bleacher::is_easy_central() const {
    return floatEqual(y, 0.950f) && (floatEqual(x, FIELD_WIDTH_M - 1.100f) || floatEqual(x, 1.100f));
}

bool Bleacher::is_easy_side(const RobotColour colour) const {
    return floatEqual(y, 0.250f) &&
           (colour == RobotColour::Blue ? floatEqual(x, FIELD_WIDTH_M - 0.775f) : floatEqual(x, 0.775f));
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
