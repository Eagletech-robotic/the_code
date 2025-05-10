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
    // myprintf("!!!! RESET DIJSKTRA !!!!\n");

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
            auto x = static_cast<uint8_t>(std::round(bleacher.x / SQUARE_SIZE_M));
            auto y = static_cast<uint8_t>(std::round(bleacher.y / SQUARE_SIZE_M));
            pqueue_.emplace(0, x, y);
        }
    } else if (target_ == TargetType::StagingWaypoint) {
    } else if (target_ == TargetType::BuildingAreaWaypoint) {
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

            if (newX < 0 || newX >= FIELD_WIDTH_SQ || newY < 0 || newY >= FIELD_HEIGHT_SQ) [[unlikely]]
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
        float const orientation = object.orientation_deg;
        myprintf("BL IN PKT: ox:%d, oy:%d, x:%.2f, y:%.2f, orientation:%.1f\n", object.x_cm, object.y_cm, x, y,
                 orientation);
        bleachers_.push_back({x, y, orientation});
        if (bleachers_.full())
            break;
    }

    // Force the recalculation of the potential field
    reset_dijkstra();
}

void World::potential_field_descent(float x, float y, float &out_speed, float &out_yaw_deg) const {
    constexpr int LOOKAHEAD_DISTANCE = 5; // In squares
    constexpr float SLOPE_THRESHOLD = 1.0f;
    constexpr float MAX_SPEED = 1.0f; // m/s

    int const i = static_cast<int>(std::floor(x / SQUARE_SIZE_M));
    int const j = static_cast<int>(std::floor(y / SQUARE_SIZE_M));

    const auto &potential = potential_ready();
    float const dx = potential[i + LOOKAHEAD_DISTANCE][j] - potential[i - LOOKAHEAD_DISTANCE][j];
    float const dy = potential[i][j + LOOKAHEAD_DISTANCE] - potential[i][j - LOOKAHEAD_DISTANCE];

    if (std::abs(dx) / LOOKAHEAD_DISTANCE <= SLOPE_THRESHOLD && std::abs(dy) / LOOKAHEAD_DISTANCE <= SLOPE_THRESHOLD) {
        myprintf("STOPPING because slope is too flat - dx: %f, dy: %f", dx, dy);
        out_speed = 0.f;
        out_yaw_deg = 0.f;
    } else {
        out_speed = MAX_SPEED;
        out_yaw_deg = std::atan2(-dy, -dx) / static_cast<float>(M_PI) * 180.0f;
        myprintf("Target angle: %f\n", out_yaw_deg);
    }
}
