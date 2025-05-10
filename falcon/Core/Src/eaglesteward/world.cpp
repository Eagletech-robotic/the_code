#include "eaglesteward/world.hpp"

#include "utils/myprintf.hpp"

#include <cfloat>
#include <cmath>

World::World() { reset(); }

void World::reset() {
    init_default_bleachers();

    /*reset_dijkstra();
    partial_compute_dijkstra();*/
}

void World::init_default_bleachers() {
    bleachers_ = {
        {0.075f, 0.400f, M_PI_2},       {0.075f, 1.325f, M_PI_2},       {0.775f, 0.250f, 0.f},
        {0.825f, 1.725f, 0.f},          {1.100f, 0.950f, 0.f},

        {3.f - 0.075f, 0.400f, M_PI_2}, {3.f - 0.075f, 1.325f, M_PI_2}, {3.f - 0.775f, 0.250f, 0.f},
        {3.f - 0.825f, 1.725f, 0.f},    {3.f - 1.100f, 0.950f, 0.f},
    };
}

void World::reset_dijkstra() {
    for (auto &row : potential_calculating()) {
        std::fill(row.begin(), row.end(), FLT_MAX);
    }

    pqueue_.clear();
    for (const auto &bleacher : bleachers_) {
        auto x = static_cast<uint8_t>(bleacher.x / SQUARE_SIZE_M);
        auto y = static_cast<uint8_t>(bleacher.y / SQUARE_SIZE_M);
        pqueue_.emplace(0, x, y);
    }
}

void World::partial_compute_dijkstra() {
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

    while (!pqueue_.empty()) {
        const PQueueNode &node = pqueue_.top();

        const auto x = node.x;
        const auto y = node.y;
        const auto baseDist = node.distance;

        pqueue_.pop();

        for (const Step &step : steps) {
            const int newX = x + step.dx;
            const int newY = y + step.dy;

            if (newX < 0 || newX >= FIELD_WIDTH_SQ || newY < 0 || newY >= FIELD_HEIGHT_SQ)
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

    ready_field_ = 1 - ready_field_;
}

void World::reset_from_eagle_packet(const EaglePacket &packet) {
    // Reset objects
    // bleachers_.clear();

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
        // bleachers_.push_back({x, y, orientation});
        // if (bleachers_.full())
        //     break;
    }
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
