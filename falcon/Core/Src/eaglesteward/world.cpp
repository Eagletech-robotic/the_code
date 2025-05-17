#include "eaglesteward/world.hpp"

#include "utils/myprintf.hpp"

#include "iot01A/top_driver.h"
#include <cfloat>
#include <cmath>

#include "utils/angles.hpp"

World::World(RobotColour colour) {
    colour_ = colour;

    // Default bleachers
    bleachers_ = {
        {0.075f, 0.400f, 0.0f},
        {0.075f, 1.325f, 0.0f},
        {0.775f, 0.250f, M_PI_2},
        {0.825f, 1.725f, M_PI_2},
        {1.100f, 0.950f, M_PI_2},
        {FIELD_WIDTH_M - 0.075f, 0.400f, 0.0f},
        {FIELD_WIDTH_M - 0.075f, 1.325f, 0.0f},
        {FIELD_WIDTH_M - 0.775f, 0.250f, M_PI_2},
        {FIELD_WIDTH_M - 0.825f, 1.725f, M_PI_2},
        {FIELD_WIDTH_M - 1.100f, 0.950f, M_PI_2},
    };

    building_areas_ = {
        {0.775f, 0.075f, M_PI_2, BuildingArea::Type::Small, RobotColour::Yellow},
        {1.225f, 0.225f, M_PI_2, BuildingArea::Type::Large, RobotColour::Yellow},
        {2.775f, 0.075f, M_PI_2, BuildingArea::Type::Small, RobotColour::Yellow},
        {2.775f, 0.875f, -M_PI, BuildingArea::Type::Large, RobotColour::Yellow},
        {FIELD_WIDTH_M - 0.775f, 0.075f, M_PI_2, BuildingArea::Type::Small, RobotColour::Blue},
        {FIELD_WIDTH_M - 1.225f, 0.225f, M_PI_2, BuildingArea::Type::Large, RobotColour::Blue},
        {FIELD_WIDTH_M - 2.775f, 0.075f, M_PI_2, BuildingArea::Type::Small, RobotColour::Blue},
        {FIELD_WIDTH_M - 2.775f, 0.875f, 0, BuildingArea::Type::Large, RobotColour::Blue},
    };

    // Pre-compute the potential field for the bleachers, as this will be our first target when the game starts.
    set_target(TargetType::BleacherWaypoint);
    while (do_some_calculations([]() { return true; }))
        ;
}

void World::set_target(TargetType new_target) {
    if (new_target == target_)
        return;

    target_ = new_target;
    reset_dijkstra();
}

bool World::remove_bleacher(const Bleacher &bleacher) {
    bool const has_removed = bleachers_.remove(bleacher);
    if (has_removed) {
        reset_dijkstra();
    }
    return has_removed;
}

void World::reset_dijkstra() {
    // Clear the potential field and the queue
    for (auto &row : potential_calculating()) {
        std::fill(row.begin(), row.end(), FLT_MAX);
    }
    pqueue_.clear();

    auto enqueue_grid_cell = [this](float x, float y) {
        if (is_in_field(x, y)) {
            auto i = static_cast<uint8_t>(std::round(x / SQUARE_SIZE_M));
            auto j = static_cast<uint8_t>(std::round(y / SQUARE_SIZE_M));
            pqueue_.emplace(0, i, j);
        }
    };

    // Add targets to the queue
    if (target_ == TargetType::BleacherWaypoint) {
        for (const auto &bleacher : bleachers_) {
            for (auto [x, y] : bleacher.waypoints()) {
                enqueue_grid_cell(x, y);
            }
        }
    }

    if (target_ == TargetType::BackstageWaypoint) {
        if (colour_ == RobotColour::Yellow) {
            enqueue_grid_cell(0.35f, 1.40f);
        } else {
            enqueue_grid_cell(FIELD_WIDTH_M - 0.35f, 1.40f);
        }
    }

    if (target_ == TargetType::BuildingAreaWaypoint) {
        for (auto &building_area : building_areas_) {
            if (building_area.is_full() || building_area.colour != colour_)
                continue;
            auto [x, y] = building_area.waypoint();
            enqueue_grid_cell(x, y);
        }
    }

    myprintf("!!!! RESET DIJSKTRA - queue size: %lu\n", pqueue_.size());
}

bool World::do_some_calculations(const std::function<bool()> &can_continue) {
    return partial_compute_dijkstra(can_continue);
}

void World::do_all_calculations_LONG() {
    while (do_some_calculations([]() { return true; }))
        ;
}

bool World::partial_compute_dijkstra(const std::function<bool()> &can_continue) {
    if (pqueue_.empty())
        return false;

    constexpr int CHECK_INTERVAL = 200;

    constexpr float COST_STRAIGHT = SQUARE_SIZE_M;
    constexpr float COST_DIAG = SQUARE_SIZE_M * 1.414f;

    constexpr float MOVABLE_OBSTACLE_COST_MULTIPLIER = 2.5f;

    struct Step {
        int dx, dy;
        float cost;
    };
    static constexpr Step steps[8] = {{-1, -1, COST_DIAG},    {0, -1, COST_STRAIGHT}, {1, -1, COST_DIAG},
                                      {-1, 0, COST_STRAIGHT}, {1, 0, COST_STRAIGHT},  {-1, 1, COST_DIAG},
                                      {0, 1, COST_STRAIGHT},  {1, 1, COST_DIAG}};

    /*const int MAX_DISTANCE = std::ceil(std::max(FIELD_WIDTH_SQ, FIELD_HEIGHT_SQ) *
                                           std::max({COST_STRAIGHT, COST_DIAG, COST_MOVABLE_OBSTACLE}));*/

    const ObstacleType *obstaclesField = obstacles_field_[ready_field_].data();
    float *potential = potential_calculating().data()->data();

    for (int it = 0; !pqueue_.empty(); ++it) {
        if (it == CHECK_INTERVAL) [[unlikely]] {
            if (can_continue()) {
                it = 0;
            } else {
                return true;
            }
        }

        const PQueueNode &node = pqueue_.top();

        const auto x = node.x;
        const auto y = node.y;
        const auto baseDist = node.distance;

        pqueue_.pop();

        for (const Step &step : steps) {
            const int newX = x + step.dx;
            const int newY = y + step.dy;

            if (static_cast<uint>(newX) >= FIELD_WIDTH_SQ || static_cast<uint>(newY) >= FIELD_HEIGHT_SQ) [[unlikely]]
                continue;

            const int newIdx = newX * FIELD_HEIGHT_SQ + newY;

            if (obstaclesField[newIdx] == ObstacleType::Fixed) [[unlikely]] {
                continue;
            }

            float stepCost = step.cost;

            if (obstaclesField[newIdx] == ObstacleType::Movable) [[unlikely]] {
                stepCost *= MOVABLE_OBSTACLE_COST_MULTIPLIER;
            }

            float cost = stepCost + baseDist;

            float &distSquare = potential[newIdx]; // Take a reference to the element in the potential field
            if (distSquare > cost) {
                distSquare = cost;
                pqueue_.emplace(distSquare, newX, newY);
            }
        }
    }

    myprintf("!! COMPLETED DIJKSTRA\n");
    ready_field_ ^= 1;
    return false;
}

void World::update_from_eagle_packet(const EaglePacket &packet) {
    colour_ = packet.robot_colour;

    // Reset objects
    bleachers_.clear();

    // Insert objects from the packet
    for (uint8_t i = 0; i < packet.object_count; ++i) {
        const auto &object = packet.objects[i];
        if (object.type != ObjectType::Bleacher)
            continue;

        float const x = object.x_cm * 0.01f;
        float const y = object.y_cm * 0.01f;
        float const orientation = object.orientation_deg * M_PI / 180.0f;
        bleachers_.push_back({x, y, orientation});
        if (bleachers_.full())
            break;
    }

    // Force the recalculation of the potential field
    reset_dijkstra();
}

void World::potential_field_descent(float x, float y, bool &out_is_local_minimum, float &out_yaw) const {
    constexpr int LOOKAHEAD_DISTANCE = 1; // In squares
    constexpr float SLOPE_THRESHOLD = 0.01f;

    int const i = static_cast<int>(std::round(x / SQUARE_SIZE_M));
    int const j = static_cast<int>(std::round(y / SQUARE_SIZE_M));

    const auto &potential = potential_ready();
    float const dx = potential[i + LOOKAHEAD_DISTANCE][j] - potential[i - LOOKAHEAD_DISTANCE][j];
    float const dy = potential[i][j + LOOKAHEAD_DISTANCE] - potential[i][j - LOOKAHEAD_DISTANCE];

    if (std::abs(dx) / LOOKAHEAD_DISTANCE <= SLOPE_THRESHOLD && std::abs(dy) / LOOKAHEAD_DISTANCE <= SLOPE_THRESHOLD) {
        myprintf("Reached local minimum");
        out_is_local_minimum = true;
        out_yaw = 0.f;
    } else {
        out_is_local_minimum = false;
        out_yaw = std::atan2(-dy, -dx);
        myprintf("Target angle: %f - distance: %f\n", to_degrees(out_yaw), potential[i][j]);
    }
}

/**
 * Confident bleachers are returned first. If there are no confident bleachers, the closest uncertain bleacher is
 * returned.
 */
std::pair<Bleacher, float> World::closest_available_bleacher(float x, float y) const {
    Bleacher best_confident;
    float best_confident_dist = std::numeric_limits<float>::max();
    Bleacher best_uncertain;
    float best_uncertain_dist = std::numeric_limits<float>::max();

    for (auto const &bleacher : bleachers_) {
        if (bleacher.in_building_area(building_areas_))
            continue;
        auto const dx = x - bleacher.x;
        auto const dy = y - bleacher.y;
        auto const distance = std::sqrt(dx * dx + dy * dy);

        if (!bleacher.uncertain) {
            if (distance < best_confident_dist) {
                best_confident_dist = distance;
                best_confident = bleacher;
            }
        } else {
            if (distance < best_uncertain_dist) {
                best_uncertain_dist = distance;
                best_uncertain = bleacher;
            }
        }
    }

    if (best_confident_dist < std::numeric_limits<float>::max())
        return {best_confident, best_confident_dist};

    if (best_uncertain_dist < std::numeric_limits<float>::max())
        return {best_uncertain, best_uncertain_dist};

    // no bleacher found
    return {Bleacher{}, std::numeric_limits<float>::max()};
}

std::pair<Bleacher, float> World::closest_dropped_bleacher(float x, float y) const {
    Bleacher best;
    float best_distance = std::numeric_limits<float>::max();

    for (const auto &bleacher : bleachers_) {
        if (!bleacher.in_building_area(building_areas_))
            continue;
        auto const dx = x - bleacher.x;
        auto const dy = y - bleacher.y;
        auto const distance = std::sqrt(dx * dx + dy * dy);
        if (distance < best_distance) {
            best_distance = distance, best = bleacher;
        }
    }

    return {best, best_distance};
}

std::pair<BuildingArea, float> World::closest_available_building_area(float x, float y) const {
    BuildingArea best;
    float best_distance = std::numeric_limits<float>::max();

    for (const auto &building_area : building_areas_) {
        if (building_area.is_full() || building_area.colour != colour_)
            continue;
        auto const [slot_x, slot_y] = building_area.available_slot();
        float const dx = x - slot_x;
        float const dy = y - slot_y;
        float const distance = std::sqrt(dx * dx + dy * dy);
        if (distance < best_distance) {
            best_distance = distance;
            best = building_area;
        }
    }

    return {best, best_distance};
}

bool World::is_in_field(float x, float y) { return x >= 0 && x < FIELD_WIDTH_M && y >= 0 && y < FIELD_HEIGHT_M; }

bool World::is_in_field_square(int i, int j) { return i >= 0 && i < FIELD_WIDTH_SQ && j >= 0 && j < FIELD_HEIGHT_SQ; }
