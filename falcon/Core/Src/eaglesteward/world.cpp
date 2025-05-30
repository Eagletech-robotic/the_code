#include "eaglesteward/world.hpp"

#include "utils/myprintf.hpp"

#include "iot01A/top_driver.h"
#include <cfloat>
#include <cmath>

#include "eaglesteward/constants.hpp"
#include "utils/angles.hpp"

World::World(RobotColour colour) {
    colour_ = colour;

    // Backstage positions
    backstages_ = {
        {0.375f, 1.775f, -M_PI_2, RobotColour::Yellow},
        {FIELD_WIDTH_M - 0.375f, 1.775f, M_PI_2, RobotColour::Blue},
    };

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
    set_target(TargetType::BleacherWaypoint, 0);
    while (do_some_calculations([]() { return true; }))
        ;
}

void World::set_target(TargetType new_target, float elapsed_time) {
    if (new_target == target_)
        return;

    target_ = new_target;
    reset_dijkstra(elapsed_time);
}

void World::reset_dijkstra(float elapsed_time) {
    // Clear previous calculations
    potential_calculating().clear();

    for (auto &row : obstacles_field_) {
        std::fill(row.begin(), row.end(), ObstacleType::None);
    }
    pqueue_.clear();

    // Add targets
    enqueue_targets();

    // Add obstacles
    GamePhase const phase = current_phase(elapsed_time);
    setup_obstacles_field(phase);

    // printf("DIJSK RST %d\n", static_cast<int>(pqueue_.size()));
}

void World::enqueue_targets() {
    auto enqueue_grid_cell = [this](float x, float y, float value = 0.0f) {
        if (is_in_field(x, y)) {
            auto i = static_cast<uint8_t>(std::round(x / SQUARE_SIZE_M));
            auto j = static_cast<uint8_t>(std::round(y / SQUARE_SIZE_M));
            pqueue_.emplace(value, i, j);
            potential_calculating().set_cell(i, j, value);
        }
    };

    if (target_ == TargetType::BleacherWaypoint) {
        for (const auto &bleacher : bleachers_) {
            if (!bleacher.initial_position)
                continue;

            if ((bleacher.is_reserved(RobotColour::Blue) && colour_ == RobotColour::Yellow) ||
                (bleacher.is_reserved(RobotColour::Yellow) && colour_ == RobotColour::Blue))
                continue;

            float value = 1.50f;
            if (bleacher.is_easy_side(colour_)) {
                value = 0.00f;
            } else if (bleacher.is_easy_central()) {
                value = 0.50f;
            } else if (bleacher.is_next_to_backstage()) {
                value = 2.50f;
            }
            for (const auto waypoint : bleacher.waypoints()) {
                enqueue_grid_cell(waypoint.x, waypoint.y, value);
            }
        }
    }

    if (target_ == TargetType::BackstageWaypoint) {
        // Waypoint just down the backstage line
        if (colour_ == RobotColour::Yellow) {
            enqueue_grid_cell(0.35f, 1.35f);
        } else {
            enqueue_grid_cell(FIELD_WIDTH_M - 0.35f, 1.35f);
        }
        // Make our robot head to the center if the waypoint is not immediately accessible
        enqueue_grid_cell(1.5f, 0.9f, 1e3);
    }

    if (target_ == TargetType::BuildingAreaWaypoint) {
        for (auto &building_area : building_areas_) {
            if (building_area.is_full() || building_area.colour != colour_)
                continue;
            const auto waypoint = building_area.waypoint();
            enqueue_grid_cell(waypoint.x, waypoint.y);
        }
    }

    if (target_ == TargetType::Evade) {
        enqueue_grid_cell(0.75, 1.00f, 0.0f);
        enqueue_grid_cell(FIELD_WIDTH_M - 0.75f, 1.00f, 0.0f);
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

void World::setup_obstacles_field(GamePhase phase) {
    // bevel = 0 => no padding
    // bevel > 0 => padding of ROBOT_RADIUS, with a bevel at each corner
    auto mark_rectangle = [this](float x_min, float x_max, float y_min, float y_max, ObstacleType type,
                                 int bevel = 0) {
        // Adjust bounds if padding is requested
        float actual_x_min = bevel > 0 ? x_min - ROBOT_RADIUS : x_min;
        float actual_x_max = bevel > 0 ? x_max + ROBOT_RADIUS : x_max;
        float actual_y_min = bevel > 0 ? y_min - ROBOT_RADIUS : y_min;
        float actual_y_max = bevel > 0 ? y_max + ROBOT_RADIUS : y_max;

        // Convert to grid coordinates
        int i0 = std::max(0, static_cast<int>(std::floor(actual_x_min / SQUARE_SIZE_M)));
        int i1 = std::min(FIELD_WIDTH_SQ - 1, static_cast<int>(std::floor(actual_x_max / SQUARE_SIZE_M)));
        int j0 = std::max(0, static_cast<int>(std::floor(actual_y_min / SQUARE_SIZE_M)));
        int j1 = std::min(FIELD_HEIGHT_SQ - 1, static_cast<int>(std::floor(actual_y_max / SQUARE_SIZE_M)));

        for (int i = i0; i <= i1; ++i) {
            for (int j = j0; j <= j1; ++j) {
                // Skip beveled corners if padding is enabled
                if (bevel > 0) {
                    // Bottom-left corner
                    if ((i - i0) + (j - j0) < bevel)
                        continue;
                    // Bottom-right corner
                    if ((i1 - i) + (j - j0) < bevel)
                        continue;
                    // Top-left corner
                    if ((i - i0) + (j1 - j) < bevel)
                        continue;
                    // Top-right corner
                    if ((i1 - i) + (j1 - j) < bevel)
                        continue;
                }

                obstacles_field_[i][j] = std::max(obstacles_field_[i][j], type);
            }
        }
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
    mark_rectangle(1.05f, 3.00f - 1.05f, 1.55f, 2.00f, ObstacleType::Fixed, 2);

    // Lateral ramps
    mark_rectangle(0.65f, 1.05f, 1.80f, 2.00f, ObstacleType::Fixed, 2);
    mark_rectangle(3.00f - 1.05f, 3.00f - 0.65f, 1.80f, 2.00f, ObstacleType::Fixed, 2);

    // Opponent reserved bleacher
    if (colour_ == RobotColour::Blue) {
        mark_rectangle(0.60f, 1.05f, 1.675f, 1.80f, ObstacleType::Fixed, 2);
    } else {
        mark_rectangle(3.0f - 1.05f, 3.00f - 0.60f, 1.675f, 1.80f, ObstacleType::Fixed, 2);
    }

    // Opponent backstage area
    if (colour_ == RobotColour::Blue) {
        mark_rectangle(0.00f, 0.60f, 1.55f, 2.00f, ObstacleType::Fixed, 2);
    } else {
        mark_rectangle(3.00f - 0.60f, 3.00f, 1.55f, 2.00f, ObstacleType::Fixed, 2);
    }

    // ---------------
    // PAMI exclusion zone
    // ---------------
    if (phase == GamePhase::PamiStarted) {
        mark_rectangle(0.95f, 2.05f, 1.20f, 1.55f, ObstacleType::Fixed, 5);
    }

    // ---------------
    // Building areas
    // ---------------
    for (const auto &building_area : building_areas_) {
        auto const occupied_space_only = building_area.colour == colour_;
        float const half_width = building_area.span_x(occupied_space_only) / 2;
        float const half_height = building_area.span_y(occupied_space_only) / 2;

        if (half_height < 0.01f || half_width < 0.01f) {
            continue;
        }

        mark_rectangle(building_area.x - half_width, building_area.x + half_width, building_area.y - half_height,
                       building_area.y + half_height, ObstacleType::Fixed, 3);
    }

    // ---------------
    // Opponent robot
    // ---------------
    // Short range interdiction
    if (target_ == TargetType::Evade) {
        mark_circle(opponent_x, opponent_y, ROBOT_RADIUS * 1.5f, ObstacleType::Fixed);
    } else {
        mark_circle(opponent_x, opponent_y, ROBOT_RADIUS * 1.5f, ObstacleType::Fixed);
    }

    // Long range repelling
    if (opponent_tracker.is_alive()) {
        mark_circle(opponent_x, opponent_y, ROBOT_RADIUS * 3.0f, ObstacleType::Movable);
    }

    // ---------------
    // Bleachers in their initial position
    // ---------------
    for (const auto &bleacher : bleachers_) {
        if (!bleacher.initial_position)
            continue;

        if (bleacher.is_easy_central() && phase == GamePhase::PamiStarted)
            // Ignore central bleachers at the end of the game, as the PAMI exclusion zone could leave us trapped
            continue;

        constexpr float HALF_SMALL = BLEACHER_WIDTH / 2.0f;
        constexpr float HALF_LARGE = BLEACHER_LENGTH / 2.0f;
        if (bleacher.is_horizontal()) {
            mark_rectangle(bleacher.x - HALF_SMALL, bleacher.x + HALF_SMALL, bleacher.y - HALF_LARGE,
                           bleacher.y + HALF_LARGE, ObstacleType::Fixed, 2);
        } else {
            mark_rectangle(bleacher.x - HALF_LARGE, bleacher.x + HALF_LARGE, bleacher.y - HALF_SMALL,
                           bleacher.y + HALF_SMALL, ObstacleType::Fixed, 2);
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
        // printf("DIJSK DONE\n");
        ready_field_ ^= 1;
    }
    return more_work;
}

void World::do_all_calculations_LONG() {
    while (do_some_calculations([]() { return true; }))
        ;
}

int World::closest_initial_bleacher_index(float x, float y) const {
    int best_index = -1;
    float best_distance = std::numeric_limits<float>::max();

    for (size_t i = 0; i < default_bleachers_.size(); ++i) {
        const auto &bleacher = default_bleachers_[i];
        float dx = x - bleacher.x;
        float dy = y - bleacher.y;
        float distance = std::sqrt(dx * dx + dy * dy);

        if (distance < best_distance) {
            best_distance = distance;
            best_index = static_cast<int>(i);
        }
    }

    return best_index;
}

void World::update_from_eagle_packet(const EaglePacket &packet, float elapsed_time) {
    colour_ = packet.robot_colour;

    if (packet.opponent_detected) {
        // Read the opponent's position and orientation
        opponent_x = static_cast<float>(packet.opponent_x_cm) / 100.0f;
        opponent_y = static_cast<float>(packet.opponent_y_cm) / 100.0f;
        opponent_theta = angle_normalize(to_radians(packet.opponent_theta_deg));
        opponent_tracker.push(true, elapsed_time, opponent_x, opponent_y);
    } else {
        opponent_tracker.push(false, elapsed_time, 0.0f, 0.0f);

        // Assume the opponent robot is in the invisible camera spot if it's not detected for several frames.
        if (opponent_tracker.get_consecutive_non_detections() >= 10) {
            opponent_x = 1.50f;
            opponent_y = 2.50f; // Out of the field is fine as the invisible spot is the opponent's arrival area.
        }
    }

    // Reset objects
    // bleachers_.clear();
    cans_.clear();
    planks_.clear();

    // Force the recalculation of the potential field
    reset_dijkstra(elapsed_time);
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

const Backstage *World::backstage() const {
    for (auto &backstage : backstages_) {
        if (backstage.colour == colour_)
            return &backstage;
    }
    return nullptr;
}

bool World::is_in_field(float x, float y) { return x >= 0 && x < FIELD_WIDTH_M && y >= 0 && y < FIELD_HEIGHT_M; }

bool World::is_in_field_square(int i, int j) { return i >= 0 && i < FIELD_WIDTH_SQ && j >= 0 && j < FIELD_HEIGHT_SQ; }

GamePhase World::current_phase(float elapsed_time) {
    return (elapsed_time >= 85.0f) ? GamePhase::PamiStarted : GamePhase::Default;
}
