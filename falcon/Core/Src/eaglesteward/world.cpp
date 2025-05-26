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

void World::reset_dijkstra() {
    // Clear previous calculations
    potential_calculating().clear();

    for (auto &row : obstacles_field_) {
        std::fill(row.begin(), row.end(), ObstacleType::None);
    }
    pqueue_.clear();

    // Add targets
    enqueue_targets();

    // Add obstacles
    setup_obstacles_field();

    printf("DIJSK RST %d\n", static_cast<int>(pqueue_.size()));
}

void World::enqueue_targets() {
    auto enqueue_grid_cell = [this](float x, float y) {
        if (is_in_field(x, y)) {
            auto i = static_cast<uint8_t>(std::round(x / SQUARE_SIZE_M));
            auto j = static_cast<uint8_t>(std::round(y / SQUARE_SIZE_M));
            pqueue_.emplace(0, i, j);
            potential_calculating().set_cell(i, j, 0);
        }
    };

    // Add targets to the queue
    if (target_ == TargetType::BleacherWaypoint) {
        for (const auto &bleacher : bleachers_) {
            if (!bleacher.initial_position)
                continue;
            for (const auto waypoint : bleacher.waypoints()) {
                enqueue_grid_cell(waypoint.x, waypoint.y);
            }
        }
    }

    if (target_ == TargetType::BackstageWaypoint) {
        if (colour_ == RobotColour::Yellow) {
            enqueue_grid_cell(0.35f, 1.35f);
        } else {
            enqueue_grid_cell(FIELD_WIDTH_M - 0.35f, 1.35f);
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

    auto mark_rectangle_with_padding = [this, mark_rectangle](float x_min, float x_max, float y_min, float y_max,
                                                              ObstacleType type) {
        // Center rectangle + top and bottom
        mark_rectangle(x_min, x_max, y_min - ROBOT_RADIUS, y_max + ROBOT_RADIUS, type);
        // Left and right sides
        mark_rectangle(x_min - ROBOT_RADIUS, x_min, y_min, y_max, type);
        mark_rectangle(x_max, x_max + ROBOT_RADIUS, y_min, y_max, type);

        // Mark quarter circles at corners
        auto mark_quarter_circle = [this](float center_x, float center_y, float radius_f, int x_dir, int y_dir,
                                          ObstacleType type) {
            // Calculate bounds in floating point first to avoid rounding errors
            float x_min_f = (x_dir < 0) ? center_x - radius_f : center_x;
            float x_max_f = (x_dir > 0) ? center_x + radius_f : center_x;
            float y_min_f = (y_dir < 0) ? center_y - radius_f : center_y;
            float y_max_f = (y_dir > 0) ? center_y + radius_f : center_y;

            // Convert to grid coordinates with proper bounds checking
            int i_start = std::max(0, static_cast<int>(std::floor(x_min_f / SQUARE_SIZE_M)));
            int i_end = std::min(FIELD_WIDTH_SQ - 1, static_cast<int>(std::ceil(x_max_f / SQUARE_SIZE_M)));
            int j_start = std::max(0, static_cast<int>(std::floor(y_min_f / SQUARE_SIZE_M)));
            int j_end = std::min(FIELD_HEIGHT_SQ - 1, static_cast<int>(std::ceil(y_max_f / SQUARE_SIZE_M)));

            // Convert center to grid coordinates
            int center_i = static_cast<int>(std::floor(center_x / SQUARE_SIZE_M));
            int center_j = static_cast<int>(std::floor(center_y / SQUARE_SIZE_M));

            // Square of radius in grid units
            float radius_grid = radius_f / SQUARE_SIZE_M;
            float square_radius_grid = radius_grid * radius_grid;

            for (int i = i_start; i <= i_end; ++i) {
                int di = i - center_i;
                float square_i = static_cast<float>(di * di);

                // Early termination if we're outside the circle radius
                if (square_i > square_radius_grid)
                    continue;

                for (int j = j_start; j <= j_end; ++j) {
                    int dj = j - center_j;

                    // Check if we're within the circle and in the correct quadrant
                    if (di * x_dir >= 0 && dj * y_dir >= 0) {
                        float square_j = static_cast<float>(dj * dj);
                        if (square_i + square_j <= square_radius_grid) {
                            obstacles_field_[i][j] = std::max(obstacles_field_[i][j], type);
                        }
                    }
                }
            }
        };

        // Corners
        mark_quarter_circle(x_min, y_min, ROBOT_RADIUS, -1, -1, type); // Bottom-left
        mark_quarter_circle(x_max, y_min, ROBOT_RADIUS, 1, -1, type);  // Bottom-right
        mark_quarter_circle(x_min, y_max, ROBOT_RADIUS, -1, 1, type);  // Top-left
        mark_quarter_circle(x_max, y_max, ROBOT_RADIUS, 1, 1, type);   // Top-right
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
    mark_circle(opponent_x, opponent_y, ROBOT_RADIUS * 4, ObstacleType::Movable);
    mark_circle(opponent_x, opponent_y, ROBOT_RADIUS * 2, ObstacleType::Fixed);

    // ---------------
    // Bleachers in their initial position
    // ---------------
    for (const auto &bleacher : bleachers_) {
        if (!bleacher.initial_position)
            continue;

        constexpr float HALF_SMALL = BLEACHER_WIDTH / 2.0f;
        constexpr float HALF_LARGE = BLEACHER_LENGTH / 2.0f;
        if (bleacher.is_horizontal()) {
            mark_rectangle_with_padding(bleacher.x - HALF_SMALL, bleacher.x + HALF_SMALL, bleacher.y - HALF_LARGE,
                                        bleacher.y + HALF_LARGE, ObstacleType::Fixed);
        } else {
            mark_rectangle_with_padding(bleacher.x - HALF_LARGE, bleacher.x + HALF_LARGE, bleacher.y - HALF_SMALL,
                                        bleacher.y + HALF_SMALL, ObstacleType::Fixed);
        }
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
    if (pqueue_.empty())
        return false;

    bool more_work = potential_calculating().compute_dijkstra_partial(obstacles_field_, pqueue_, can_continue);
    if (!more_work) {
        // Computation is done, swap buffers
        printf("DIJSK DONE\n");
        ready_field_ ^= 1;
    }
    return more_work;
}

void World::do_all_calculations_LONG() {
    while (do_some_calculations([]() { return true; }))
        ;
}

void World::update_from_eagle_packet(const EaglePacket &packet) {
    colour_ = packet.robot_colour;

    // Reset objects
    // bleachers_.clear();
    cans_.clear();
    planks_.clear();

    // 1) Insert initial bleachers from  the header
    // for (size_t i = 0; i < 10; ++i) {
    // if (packet.initial_bleachers[i]) {
    // bleachers_.push_back(default_bleachers_[i]);
    // }
    // }

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

std::pair<Bleacher *, float> World::closest_available_bleacher(float x, float y) {
    Bleacher *best = nullptr;
    float best_distance = std::numeric_limits<float>::max();

    for (auto &bleacher : bleachers_) {
        if (!bleacher.initial_position)
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

void World::remove_bleacher(float x, float y) {
    Bleacher *bleacher = nullptr;
    for (auto &it : bleachers_) {
        constexpr float TOLERANCE = 0.15f; // In simulation, bleachers move when the robot pushes them during pickup.
        if (std::abs(it.x - x) < TOLERANCE && std::abs(it.y - y) < TOLERANCE) {
            bleacher = &it;
            break;
        }
    }
    if (bleacher)
        bleachers_.remove(*bleacher);
}

BuildingArea *World::closest_building_area(float x, float y, bool only_available) {
    BuildingArea *best = nullptr;
    float best_distance = std::numeric_limits<float>::max();

    for (auto &building_area : building_areas_) {
        if ((only_available && building_area.is_full()) || building_area.colour != colour_)
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

    return best;
}

bool World::is_in_field(float x, float y) { return x >= 0 && x < FIELD_WIDTH_M && y >= 0 && y < FIELD_HEIGHT_M; }

bool World::is_in_field_square(int i, int j) { return i >= 0 && i < FIELD_WIDTH_SQ && j >= 0 && j < FIELD_HEIGHT_SQ; }
