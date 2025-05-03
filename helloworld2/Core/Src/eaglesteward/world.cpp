#include "eaglesteward/world.hpp"

#include <cmath>
#include <cstdint>
#include <cstring> // std::memset

World::World() {
    init_default_bleachers();
    build_potential_field();
}

void World::init_default_bleachers() {
    bleachers_ = {
        {0.075f, 0.400f, M_PI_2},       {0.075f, 1.325f, M_PI_2},       {0.775f, 0.250f, 0.f},
        {0.825f, 1.725f, 0.f},          {1.100f, 0.950f, 0.f},

        {3.f - 0.075f, 0.400f, M_PI_2}, {3.f - 0.075f, 1.325f, M_PI_2}, {3.f - 0.775f, 0.250f, 0.f},
        {3.f - 0.825f, 1.725f, 0.f},    {3.f - 1.100f, 0.950f, 0.f},
    };
}

void World::reset_from_eagle_packet(const EaglePacket &eagle_packet) {
    // Reset objects
    bleachers_.clear();

    // Insert objects from the packet
    for (uint8_t i = 0; i < eagle_packet.object_count; ++i) {
        const auto &object = eagle_packet.objects[i];
        if (object.type != ObjectType::Bleacher)
            continue;

        float const x = object.x_cm * 0.01f;
        float const y = object.y_cm * 0.01f;
        float const orientation = object.orientation_deg;
        bleachers_.push_back({x, y, orientation});
        if (bleachers_.full())
            break;
    }

    // Rebuild the potential field
    build_potential_field();
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

void World::build_potential_field() {
    std::memset(potential_field_.data(), 0, sizeof(potential_field_));
    add_walls();
    add_bleachers();
}

/* --- walls -------------------------------------------------------- */
void World::add_walls() {
    constexpr float INF_DIST = 0.35f; // m
    constexpr float MAX_POT = 35.f;
    const int w = static_cast<int>(INF_DIST / SQUARE_SIZE_M);

    for (int i = 0; i < FIELD_WIDTH_SQ; ++i) {
        for (int j = 0; j < FIELD_HEIGHT_SQ; ++j) {
            float &cell = potential_field_[i][j];
            if (i < w)
                cell += MAX_POT * (w - i) / w;
            else if (i >= FIELD_WIDTH_SQ - w)
                cell += MAX_POT * (i - (FIELD_WIDTH_SQ - w)) / w;

            if (j < w)
                cell += MAX_POT * (w - j) / w;
            else if (j >= FIELD_HEIGHT_SQ - w)
                cell += MAX_POT * (j - (FIELD_HEIGHT_SQ - w)) / w;
        }
    }
}

/* --- bleacher --------------------------------------------- */
void World::add_bleachers() {
    for (auto &bleacher : bleachers_) {
        const auto &field = bleacher.potential_field();

        const int field_width = field.size();
        const int field_height = static_cast<int>(field[0].size());

        int bleacher_i = static_cast<int>(std::round(bleacher.x / SQUARE_SIZE_M)) - field_width / 2;
        int bleacher_j = static_cast<int>(std::round(bleacher.y / SQUARE_SIZE_M)) - field_height / 2;

        for (int field_i = 0; field_i < field_width; ++field_i)
            for (int field_j = 0; field_j < field_height; ++field_j) {
                int i = bleacher_i + field_i;
                int j = bleacher_j + field_j;
                if (i < 0 || j < 0 || i >= FIELD_WIDTH_SQ || j >= FIELD_HEIGHT_SQ)
                    continue;
                potential_field_[i][j] += field[field_i][field_j];
            }
    }
}
