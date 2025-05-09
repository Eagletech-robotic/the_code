#include "eaglesteward/world.hpp"

#include "utils/myprintf.hpp"

#include <cfloat>
#include <cmath>

World::World(RobotColour colour) {
    colour_ = colour;

    // Some default values before we receive the first packet.
    bleachers_ = {
        {0.075f, 0.400f, M_PI_2},       {0.075f, 1.325f, M_PI_2},    {0.775f, 0.250f, 0.f},
        {0.825f, 1.725f, 0.f},          {1.100f, 0.950f, 0.f},       {3.f - 0.075f, 0.400f, M_PI_2},
        {3.f - 0.075f, 1.325f, M_PI_2}, {3.f - 0.775f, 0.250f, 0.f}, {3.f - 0.825f, 1.725f, 0.f},
        {3.f - 1.100f, 0.950f, 0.f},
    };

    // Pre-compute the potential field for the bleachers, as this will be our first target when the game starts.
    set_target(TargetType::BleacherWaypoint);
    while (do_some_calculations())
        ;
}

void World::set_target(TargetType new_target) {
    if (new_target == target_)
        return;

    target_ = new_target;
    reset_dijkstra();
}

void World::reset_dijkstra() {
    myprintf("!!!! RESET DIJSKTRA !!!!\n");

    // Clear the potential field and the queue
    for (auto &row : potential_calculating()) {
        std::fill(row.begin(), row.end(), FLT_MAX);
    }
    pqueue_.clear();

    // Add the target to the queue
    if (target_ == TargetType::None) {
        return;
    } else if (target_ == TargetType::BleacherWaypoint) {
        for (const auto &bleacher : bleachers_) {
            for (const auto &[tx, ty] : bleacher_waypoints(bleacher)) {
                if (is_in_field(tx, ty)) {
                    auto i = static_cast<uint8_t>(std::round(tx / SQUARE_SIZE_M));
                    auto j = static_cast<uint8_t>(std::round(ty / SQUARE_SIZE_M));
                    pqueue_.emplace(0, i, j);
                }
            }
        }
    } else if (target_ == TargetType::BackstageWaypoint) {
        if (colour_ == RobotColour::Yellow) {
            pqueue_.emplace(0, 0.35, 1.4);
        } else if (colour_ == RobotColour::Blue) {
            pqueue_.emplace(0, 2.65, 1.4);
        }
    } else if (target_ == TargetType::BuildingAreaWaypoint) {
        if (colour_ == RobotColour::Yellow) {
            pqueue_.emplace(0, 0.775, 0.35);
            pqueue_.emplace(0, 1.225, 0.5);
            pqueue_.emplace(0, 2.775, 0.35);
            pqueue_.emplace(0, 2.5, 0.875);
        } else if (colour_ == RobotColour::Blue) {
            pqueue_.emplace(0, 3 - 0.775, 0.35);
            pqueue_.emplace(0, 3 - 1.225, 0.5);
            pqueue_.emplace(0, 3 - 2.775, 0.35);
            pqueue_.emplace(0, 3 - 2.5, 0.875);
        }
    }
}

bool World::do_some_calculations() {
    return partial_compute_dijkstra([]() { return true; });
}

bool World::partial_compute_dijkstra(const std::function<bool()> &can_continue) {
    if (pqueue_.empty())
        return false;

    constexpr int CHECK_INTERVAL = 200;

    constexpr float COST_STRAIGHT = 10.0f;
    constexpr float COST_DIAG = 14.0f;
    // constexpr float COST_MOVABLE_OBSTACLE = 50.0f;

    struct Step {
        int dx, dy;
        float cost;
    };
    static constexpr Step steps[8] = {{-1, -1, COST_DIAG},    {0, -1, COST_STRAIGHT}, {1, -1, COST_DIAG},
                                      {-1, 0, COST_STRAIGHT}, {1, 0, COST_STRAIGHT},  {-1, 1, COST_DIAG},
                                      {0, 1, COST_STRAIGHT},  {1, 1, COST_DIAG}};

    /*const int MAX_DISTANCE = std::ceil(std::max(FIELD_WIDTH_SQ, FIELD_HEIGHT_SQ) *
                                           std::max({COST_STRAIGHT, COST_DIAG, COST_MOVABLE_OBSTACLE}));*/

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

            if (!is_in_field_square(newX, newY)) [[unlikely]]
                continue;

            const float cost = step.cost + baseDist;

            const int newIdx = newX * FIELD_HEIGHT_SQ + newY;
            float &distSquare = potential[newIdx];
            if (distSquare > cost) {
                distSquare = cost;
                pqueue_.emplace(distSquare, newX, newY);
            }
        }
    }

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
        myprintf("BL IN PKT: ox:%d, oy:%d, x:%.2f, y:%.2f, orientation:%.1f\n", object.x_cm, object.y_cm, x, y,
                 orientation);
        bleachers_.push_back({x, y, orientation});
        if (bleachers_.full())
            break;
    }

    // Force the recalculation of the potential field
    reset_dijkstra();
}

void World::potential_field_descent(float x, float y, bool &is_moving, float &out_yaw_deg) const {
    constexpr int LOOKAHEAD_DISTANCE = 1; // In squares
    constexpr float SLOPE_THRESHOLD = 1.0f;

    int const i = static_cast<int>(std::floor(x / SQUARE_SIZE_M));
    int const j = static_cast<int>(std::floor(y / SQUARE_SIZE_M));

    const auto &potential = potential_ready();
    float const dx = potential[i + LOOKAHEAD_DISTANCE][j] - potential[i - LOOKAHEAD_DISTANCE][j];
    float const dy = potential[i][j + LOOKAHEAD_DISTANCE] - potential[i][j - LOOKAHEAD_DISTANCE];

    if (std::abs(dx) / LOOKAHEAD_DISTANCE <= SLOPE_THRESHOLD && std::abs(dy) / LOOKAHEAD_DISTANCE <= SLOPE_THRESHOLD) {
        myprintf("STOPPING because slope is too flat - dx: %f, dy: %f", dx, dy);
        is_moving = false;
        out_yaw_deg = 0.f;
    } else {
        is_moving = true;
        out_yaw_deg = std::atan2(-dy, -dx) / static_cast<float>(M_PI) * 180.0f;
        myprintf("Target angle: %f\n", out_yaw_deg);
    }
}

std::array<std::pair<float, float>, 2> World::bleacher_waypoints(const Bleacher &bleacher) const {
    constexpr float OFFSET = 0.30f; // 30 cm orthogonal to the bleacher
    const float nx = std::cos(bleacher.orientation + M_PI_2);
    const float ny = std::sin(bleacher.orientation + M_PI_2);
    return {{
        {bleacher.x + OFFSET * nx, bleacher.y + OFFSET * ny},
        {bleacher.x - OFFSET * nx, bleacher.y - OFFSET * ny},
    }};
}

std::pair<Bleacher, float> World::closest_bleacher(float x, float y) const {
    Bleacher best;
    float best_d = 1e9f;

    for (const auto &b : bleachers_) {
        float dx = x - b.x;
        float dy = y - b.y;
        float d = std::sqrt(dx * dx + dy * dy);
        if (d < best_d)
            best_d = d, best = b;
    }
    return {best, best_d};
}

std::pair<Bleacher, float> World::closest_bleacher_waypoint(float x, float y) const {
    Bleacher best;
    float best_d = 1e9f;

    for (const auto &b : bleachers_) {
        for (const auto &[wx, wy] : bleacher_waypoints(b)) {
            const float dx = x - wx;
            const float dy = y - wy;
            const float d = std::sqrt(dx * dx + dy * dy);
            if (d < best_d) {
                best_d = d;
                best = b;
            }
        }
    }
    return {best, best_d};
}

bool World::is_in_field(float x, float y) { return x >= 0 && x < FIELD_WIDTH_M && y >= 0 && y < FIELD_HEIGHT_M; }

bool World::is_in_field_square(int i, int j) { return i >= 0 && i < FIELD_WIDTH_SQ && j >= 0 && j < FIELD_HEIGHT_SQ; }
