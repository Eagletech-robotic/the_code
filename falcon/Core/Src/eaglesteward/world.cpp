#include "eaglesteward/world.hpp"

#include "utils/myprintf.hpp"

#include "iot01A/top_driver.h"
#include <cfloat>
#include <cmath>

#include "eaglesteward/constants.hpp"
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

void World::carry_bleacher(Bleacher &bleacher) {
    drop_carried_bleacher();
    bleacher.is_carried = true;
}

void World::drop_carried_bleacher() {
    for (auto &bleacher : bleachers_) {
        bleacher.is_carried = false;
    }
}

Bleacher *World::carried_bleacher() {
    for (auto &bleacher : bleachers_) {
        if (bleacher.is_carried)
            return &bleacher;
    }
    return nullptr;
}

void World::reset_dijkstra() {
    // Clear previous calculations
    for (auto &row : potential_calculating()) {
        std::fill(row.begin(), row.end(), FLT_MAX);
    }
    for (auto &row : obstacles_field_) {
        std::fill(row.begin(), row.end(), ObstacleType::None);
    }
    pqueue_.clear();

    // Add targets
    enqueue_targets();

    // Add obstacles
    // setup_obstacles_field();

    printf("RES_DIJSKTRA(%zu)\n", pqueue_.size());
}

void World::enqueue_targets() {
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
            if (bleacher.in_building_area(building_areas_) || bleacher.uncertain)
                continue;
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
}

void World::setup_obstacles_field() {
    auto mark_rectangle = [this](float x_min, float x_max, float y_min, float y_max, ObstacleType type, bool force) {
        int i0 = std::max(0, static_cast<int>(std::floor(x_min / SQUARE_SIZE_M)));
        int i1 = std::min(FIELD_WIDTH_SQ - 1, static_cast<int>(std::floor(x_max / SQUARE_SIZE_M)));
        int j0 = std::max(0, static_cast<int>(std::floor(y_min / SQUARE_SIZE_M)));
        int j1 = std::min(FIELD_HEIGHT_SQ - 1, static_cast<int>(std::floor(y_max / SQUARE_SIZE_M)));

        for (int i = i0; i <= i1; ++i)
            for (int j = j0; j <= j1; ++j)
                obstacles_field_[i][j] = force ? type : std::max(obstacles_field_[i][j], type);
    };

    auto mark_circle = [this](float center_x, float center_y, float radius_f, ObstacleType type) {
        int radius = static_cast<int>(std::ceil(radius_f / SQUARE_SIZE_M));
        int square_radius = radius * radius;

        int center_i = static_cast<int>(std::round(center_x / SQUARE_SIZE_M));
        int center_j = static_cast<int>(std::round(center_y / SQUARE_SIZE_M));
        int i0 = std::max(0, center_i - radius), i1 = std::min(FIELD_WIDTH_SQ - 1, center_i + radius);
        int j0 = std::max(0, center_j - radius), j1 = std::min(FIELD_HEIGHT_SQ - 1, center_j + radius);

        for (int i = i0; i <= i1; ++i) {
            int square_i = (i - center_i) * (i - center_i);
            for (int j = j0; j <= j1; ++j) {
                int dj = j - center_j;
                if (square_i + dj * dj <= square_radius)
                    obstacles_field_[i][j] = std::max(obstacles_field_[i][j], type);
            }
        }
    };

    // ---------------
    // Scene
    // ---------------
    mark_rectangle(0.f, FIELD_WIDTH_M, 1.55 - ROBOT_RADIUS, FIELD_HEIGHT_M, ObstacleType::Fixed, false);

    // Backstage corridor
    constexpr float CORRIDOR_Y_MIN = 1.55f - ROBOT_RADIUS;
    constexpr float CORRIDOR_Y_MAX = 1.80f - ROBOT_RADIUS;

    float corridor_x_min, corridor_x_max;
    if (colour_ == RobotColour::Blue) {
        corridor_x_min = 1.95f + ROBOT_RADIUS;
        corridor_x_max = 2.40f - ROBOT_RADIUS;
    } else {
        // Yellow
        corridor_x_min = 0.60f + ROBOT_RADIUS;
        corridor_x_max = 1.05f - ROBOT_RADIUS;
    }
    mark_rectangle(corridor_x_min, corridor_x_max, CORRIDOR_Y_MIN, CORRIDOR_Y_MAX, ObstacleType::None, true);

    // ---------------
    // Opponent's building areas
    // ---------------
    for (const auto &building_area : building_areas_) {
        if (building_area.colour == colour_)
            continue;
        float const half_width = building_area.span_x() / 2 + ROBOT_RADIUS;
        float const half_height = building_area.span_y() / 2 + ROBOT_RADIUS;
        mark_rectangle(building_area.x - half_width, building_area.x + half_width, building_area.y - half_height,
                       building_area.y + half_height, ObstacleType::Fixed, false);
    }

    // ---------------
    // Opponent robot
    // ---------------
    mark_circle(opponent_x, opponent_y, ROBOT_RADIUS * 3, ObstacleType::Movable);
    mark_circle(opponent_x, opponent_y, ROBOT_RADIUS * 2, ObstacleType::Fixed);

    // ---------------
    // Bleachers
    // ---------------
    for (const auto &bleacher : bleachers_) {
        if (bleacher.is_carried)
            continue;
        mark_circle(bleacher.x, bleacher.y, 0.35f, ObstacleType::Movable);
        if (!bleacher.uncertain) {
            mark_circle(bleacher.x, bleacher.y, 0.15f, ObstacleType::Fixed);
        }
    }
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

    const ObstacleType *obstaclesField = obstacles_field_.data()->data();
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

            if (static_cast<uint32_t>(newX) >= FIELD_WIDTH_SQ || static_cast<uint32_t>(newY) >= FIELD_HEIGHT_SQ)
                [[unlikely]]
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

    printf("COMP_DIJSKTRA\n");
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

    // ordre 4
    //    float const dx = (-potential[i+2][j] + 8.0*potential[i+1][j] - 8.0*potential[i-1][j] + potential[i-2][j]);
    //    float const dy = (-potential[i][j+2] + 8.0*potential[i][j+1]  - 8.0*potential[i][j-1] + potential[i][j-2]);
    float norm = sqrtf(dx * dx + dy * dy);
    // if (std::abs(dx) / LOOKAHEAD_DISTANCE <= SLOPE_THRESHOLD && std::abs(dy) / LOOKAHEAD_DISTANCE <= SLOPE_THRESHOLD)
    // {
    if (norm <= SLOPE_THRESHOLD) {
        myprintf("Reached local minimum");
        out_is_local_minimum = true;
        out_yaw = 0.f;
    } else {
        out_is_local_minimum = false;
        out_yaw = std::atan2(-dy, -dx);
        myprintf("Target angle: %f - distance: %f\n", to_degrees(out_yaw), potential[i][j]);
    }
}

std::pair<Bleacher *, float> World::closest_available_bleacher(float x, float y) {
    Bleacher *best = nullptr;
    float best_distance = std::numeric_limits<float>::max();

    for (auto &bleacher : bleachers_) {
        if (bleacher.in_building_area(building_areas_) || bleacher.uncertain)
            continue;
        auto const dx = x - bleacher.x;
        auto const dy = y - bleacher.y;
        auto const distance = std::sqrt(dx * dx + dy * dy);
        if (distance < best_distance) {
            best_distance = distance;
            best = &bleacher;
        }
    }

    if (best)
        return {best, best_distance};
    return {nullptr, std::numeric_limits<float>::max()};
}

std::pair<Bleacher *, float> World::closest_bleacher_in_building_area(float x, float y) {
    Bleacher *best = nullptr;
    float best_distance = std::numeric_limits<float>::max();

    for (auto &bleacher : bleachers_) {
        if (!bleacher.in_building_area(building_areas_))
            continue;
        auto const dx = x - bleacher.x;
        auto const dy = y - bleacher.y;
        auto const distance = std::sqrt(dx * dx + dy * dy);
        if (distance < best_distance) {
            best_distance = distance;
            best = &bleacher;
        }
    }

    return {best, best_distance};
}

std::pair<BuildingArea *, float> World::closest_available_building_area(float x, float y) {
    BuildingArea *best = nullptr;
    float best_distance = std::numeric_limits<float>::max();

    for (auto &building_area : building_areas_) {
        if (building_area.is_full() || building_area.colour != colour_)
            continue;
        auto const [slot_x, slot_y] = building_area.available_slot();
        float const dx = x - slot_x;
        float const dy = y - slot_y;
        float const distance = std::sqrt(dx * dx + dy * dy);
        if (distance < best_distance) {
            best_distance = distance;
            best = &building_area;
        }
    }

    return {best, best_distance};
}

bool World::is_in_field(float x, float y) { return x >= 0 && x < FIELD_WIDTH_M && y >= 0 && y < FIELD_HEIGHT_M; }

bool World::is_in_field_square(int i, int j) { return i >= 0 && i < FIELD_WIDTH_SQ && j >= 0 && j < FIELD_HEIGHT_SQ; }
