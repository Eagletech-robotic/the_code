#include "eaglesteward/world.hpp"

#include "utils/myprintf.hpp"

#include "iot01A/top_driver.h"
#include <cfloat>
#include <cmath>

#include "eaglesteward/constants.hpp"
#include "utils/angles.hpp"

World::World(RobotColour colour) {
    colour_ = colour;

    // Default bleachers, sorted clockwise from the center right
    default_bleachers_ = {
        {FIELD_WIDTH_M - 1.100f, 0.950f, M_PI_2, true},
        {FIELD_WIDTH_M - 0.825f, 1.725f, M_PI_2, true},
        {FIELD_WIDTH_M - 0.075f, 1.325f, 0.0f, true},
        {FIELD_WIDTH_M - 0.075f, 0.400f, 0.0f, true},
        {FIELD_WIDTH_M - 0.775f, 0.250f, M_PI_2, true},
        {0.775f, 0.250f, M_PI_2, true},
        {0.075f, 0.400f, 0.0f, true},
        {0.075f, 1.325f, 0.0f, true},
        {0.825f, 1.725f, M_PI_2, true},
        {1.100f, 0.950f, M_PI_2, true},
    };

    // Create a copy of the default bleachers; these are the actual bleachers in the world.
    for (auto default_bleacher : default_bleachers_) {
        bleachers_.push_back(default_bleacher);
    }

    // Building areas, fixed positions.
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
    setup_obstacles_field();

    printf("DIJSK RST %lu\n", pqueue_.size());
}

void World::enqueue_targets() {
    auto enqueue_grid_cell = [this](float x, float y) {
        if (is_in_field(x, y)) {
            auto i = static_cast<uint8_t>(std::round(x / SQUARE_SIZE_M));
            auto j = static_cast<uint8_t>(std::round(y / SQUARE_SIZE_M));
            pqueue_.emplace(0, i, j);
            potential_calculating()[i][j] = 0;
        }
    };

    // Add targets to the queue
    if (target_ == TargetType::BleacherWaypoint) {
        for (const auto &bleacher : bleachers_) {
            if (bleacher.in_building_area(building_areas_) || bleacher.uncertain)
                continue;
            for (const auto waypoint : bleacher.waypoints()) {
                enqueue_grid_cell(waypoint.x, waypoint.y);
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
            const auto waypoint = building_area.waypoint();
            enqueue_grid_cell(waypoint.x, waypoint.y);
        }
    }

    if (target_ == TargetType::TestPoint0)
        enqueue_grid_cell(0.75f, 0.30f);

    if (target_ == TargetType::TestPoint1)
        enqueue_grid_cell(1.5f, 0.30f);

    if (target_ == TargetType::TestPoint2)
        enqueue_grid_cell(1.5f, 1.2f);

    if (target_ == TargetType::TestPoint3)
        enqueue_grid_cell(0.75f, 1.2f);
}

void World::setup_obstacles_field() {
    auto mark_rectangle = [this](float x_min, float x_max, float y_min, float y_max, ObstacleType type) {
        int i0 = std::max(0, static_cast<int>(std::floor(x_min / SQUARE_SIZE_M)));
        int i1 = std::min(FIELD_WIDTH_SQ - 1, static_cast<int>(std::floor(x_max / SQUARE_SIZE_M)));
        int j0 = std::max(0, static_cast<int>(std::floor(y_min / SQUARE_SIZE_M)));
        int j1 = std::min(FIELD_HEIGHT_SQ - 1, static_cast<int>(std::floor(y_max / SQUARE_SIZE_M)));

        for (int i = i0; i <= i1; ++i)
            for (int j = j0; j <= j1; ++j)
                obstacles_field_[i][j] = std::max(obstacles_field_[i][j], type);
    };

    auto mark_circle = [this](float center_x, float center_y, float radius_f, ObstacleType type) {
        int radius = static_cast<int>(std::floor(radius_f / SQUARE_SIZE_M));
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

    auto mark_rectangle_with_padding = [this, mark_rectangle, mark_circle](float x_min, float x_max, float y_min,
                                                                           float y_max, ObstacleType type) {
        // mark_rectangle(x_min - ROBOT_RADIUS, x_max + ROBOT_RADIUS, y_min - ROBOT_RADIUS, y_max + ROBOT_RADIUS, type);
        mark_rectangle(x_min, x_max, y_min - ROBOT_RADIUS, y_max + ROBOT_RADIUS, type);
        mark_rectangle(x_min - ROBOT_RADIUS, x_min, y_min, y_max, type);
        mark_rectangle(x_max, x_max + ROBOT_RADIUS, y_min, y_max, type);

        mark_circle(x_min, y_min, ROBOT_RADIUS, type);
        mark_circle(x_min, y_max, ROBOT_RADIUS, type);
        mark_circle(x_max, y_min, ROBOT_RADIUS, type);
        mark_circle(x_max, y_max, ROBOT_RADIUS, type);
    };

    // ---------------
    // Borders
    // ---------------
    mark_rectangle(0.0f, FIELD_WIDTH_M, 0.0f, ROBOT_RADIUS, ObstacleType::Fixed);
    mark_rectangle(0.0f, ROBOT_RADIUS, 0.0f, FIELD_HEIGHT_M, ObstacleType::Fixed);
    mark_rectangle(3.00f - ROBOT_RADIUS, FIELD_WIDTH_M, 0.0f, FIELD_HEIGHT_M, ObstacleType::Fixed);

    // ---------------
    // Scene
    // ---------------
    // Central scene
    mark_rectangle_with_padding(1.05f, 3.00f - 1.05f, 1.55f, 2.00f, ObstacleType::Fixed);

    // Lateral ramps
    mark_rectangle_with_padding(0.65f, 1.05f, 1.80f, 2.00f, ObstacleType::Fixed);
    mark_rectangle_with_padding(3.00f - 1.05f, 3.00f - 0.65f, 1.80f, 2.00f, ObstacleType::Fixed);

    // Opponent reserved bleacher
    if (colour_ == RobotColour::Blue) {
        mark_rectangle_with_padding(0.60f, 1.05f, 1.675f, 1.80f, ObstacleType::Fixed);
    } else {
        mark_rectangle_with_padding(3.0f - 1.05f, 3.00f - 0.60f, 1.675f, 1.80f, ObstacleType::Fixed);
    }

    // Opponent backstage area
    if (colour_ == RobotColour::Blue) {
        mark_rectangle_with_padding(0.00f, 0.60f, 1.55f, 2.00f, ObstacleType::Fixed);
    } else {
        mark_rectangle_with_padding(3.00f - 0.60f, 3.00f, 1.55f, 2.00f, ObstacleType::Fixed);
    }

    // ---------------
    // Opponent's building areas
    // ---------------
    for (const auto &building_area : building_areas_) {
        if (building_area.colour == colour_)
            continue;
        float const half_width = building_area.span_x() / 2;
        float const half_height = building_area.span_y() / 2;
        mark_rectangle_with_padding(building_area.x - half_width, building_area.x + half_width,
                                    building_area.y - half_height, building_area.y + half_height, ObstacleType::Fixed);
    }

    // ---------------
    // Opponent robot
    // ---------------
    mark_circle(opponent_x, opponent_y, ROBOT_RADIUS * 3, ObstacleType::Movable);
    mark_circle(opponent_x, opponent_y, ROBOT_RADIUS * 2, ObstacleType::Fixed);

    // ---------------
    // Bleachers in their initial position
    // ---------------
    for (const auto &bleacher : bleachers_) {
        if (!bleacher.initial_position)
            continue;
        mark_circle(bleacher.x, bleacher.y, BLEACHER_LENGTH / 2.0f /*+ ROBOT_RADIUS*/, ObstacleType::Fixed);
    }

    // ---------------
    // Cans & planks
    // ---------------
    for (const auto &can : cans_) {
        mark_circle(can.x, can.y, 0.20f, ObstacleType::Movable);
        if (target_ == TargetType::BuildingAreaWaypoint) {
            mark_circle(can.x, can.y, 0.10f, ObstacleType::Fixed);
        }
    }
    for (const auto &plank : planks_) {
        mark_circle(plank.x, plank.y, 0.25f, ObstacleType::Movable);
        if (target_ == TargetType::BuildingAreaWaypoint) {
            mark_circle(plank.x, plank.y, 0.15f, ObstacleType::Fixed);
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

    printf("DIJSK DONE\n");
    ready_field_ ^= 1;
    return false;
}

void World::update_from_eagle_packet(const EaglePacket &packet) {
    colour_ = packet.robot_colour;

    // Reset objects
    bleachers_.clear();
    cans_.clear();
    planks_.clear();

    // 1) Insert initial bleachers from  the header
    for (size_t i = 0; i < 10; ++i) {
        if (packet.initial_bleachers[i]) {
            bleachers_.push_back(default_bleachers_[i]);
        }
    }

    // 2) Insert objects from the object list
    for (uint8_t i = 0; i < packet.object_count; ++i) {
        auto const &object = packet.objects[i];

        auto const x = static_cast<float>(object.x_cm) * 0.01f;
        auto const y = static_cast<float>(object.y_cm) * 0.01f;
        auto const orientation = to_radians(object.orientation_deg);

        switch (object.type) {
        case ObjectType::Bleacher:
            if (!bleachers_.full())
                bleachers_.push_back({x, y, orientation, false});
            break;
        case ObjectType::Can:
            if (!cans_.full())
                cans_.push_back({x, y, orientation});
            break;
        case ObjectType::Plank:
            if (!planks_.full())
                planks_.push_back({x, y, orientation});
            break;
        default:
            break;
        }
    }

    // Force the recalculation of the potential field
    reset_dijkstra();
}

// Returns the potential value at the given grid cell (i, j).
// If the value is infinite, return the potential of the best neighbour + distance to it.
static inline float finite_potential(const auto &P, int i, int j) {
    float const v = P[i][j];
    if (v != FLT_MAX)
        return v;

    // Look for the neighbour with the highest finite potential
    float best = -FLT_MAX;
    int best_c = 0, best_r = 0;

    for (int c = -1; c <= 1; ++c)
        for (int r = -1; r <= 1; ++r) {
            if (c == 0 && r == 0)
                continue;
            int ii = i + c;
            int jj = j + r;
            if (ii < 0 || ii >= FIELD_WIDTH_SQ || jj < 0 || jj >= FIELD_HEIGHT_SQ)
                continue;
            float neighbour = P[ii][jj];
            if (neighbour != FLT_MAX && neighbour > best) {
                best = neighbour;
                best_c = c;
                best_r = r;
            }
        }

    // all neighbours are infinite
    if (best == -FLT_MAX)
        return FLT_MAX;

    constexpr float d_cell = SQUARE_SIZE_M;
    return best + d_cell * std::sqrt(float(best_c * best_c + best_r * best_r));
}

// Renvoie V(x,y) pour des coordonnées réelles (en mètres)
float World::potential_at(float px, float py) const {
    const auto &P = potential_ready();

    // indices fractionnaires
    float gx = px / SQUARE_SIZE_M;
    float gy = py / SQUARE_SIZE_M;

    int i = static_cast<int>(std::floor(gx));
    int j = static_cast<int>(std::floor(gy));
    float tx = gx - i; // 0..1
    float ty = gy - j; // 0..1

    // bornes
    i = std::clamp(i, 0, FIELD_WIDTH_SQ - 2);
    j = std::clamp(j, 0, FIELD_HEIGHT_SQ - 2);

    // bilinéaire
    float v00 = finite_potential(P, i, j);
    float v10 = finite_potential(P, i + 1, j);
    float v01 = finite_potential(P, i, j + 1);
    float v11 = finite_potential(P, i + 1, j + 1);

    return (1 - tx) * (1 - ty) * v00 + (tx) * (1 - ty) * v10 + (1 - tx) * (ty)*v01 + (tx) * (ty)*v11;
}

void World::potential_field_descent(float x, float y, bool &out_is_local_minimum, float &out_yaw) const {
    constexpr float DELTA = 0.25f * SQUARE_SIZE_M; // pas sous-cellule
    constexpr float SLOPE_THRESHOLD = 0.01f;
    static float current_out_yaw = 0.0f;

    float dx = (potential_at(x + DELTA, y) - potential_at(x - DELTA, y)) / (2.f * DELTA);
    float dy = (potential_at(x, y + DELTA) - potential_at(x, y - DELTA)) / (2.f * DELTA);

    float norm = std::hypot(dx, dy);
    if (norm <= SLOPE_THRESHOLD) {
        out_is_local_minimum = true;
        out_yaw = current_out_yaw;
    } else {
        out_is_local_minimum = false;
        out_yaw = std::atan2(-dy, -dx);
        current_out_yaw = 0.99f * current_out_yaw + 0.01f * out_yaw;
        out_yaw = current_out_yaw;
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
        auto const slot = building_area.available_slot();
        float const dx = x - slot.x;
        float const dy = y - slot.y;
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
