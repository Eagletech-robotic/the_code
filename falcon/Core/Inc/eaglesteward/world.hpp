#pragma once

#include "robotic/eagle_packet.hpp"
#include "utils/bounded_pqueue.hpp"
#include "utils/game_entities.hpp"
#include "utils/sized_array.hpp"

#include <array>
#include <utility>

struct PQueueNode {
    float distance;
    uint8_t x;
    uint8_t y;

    bool operator<(const PQueueNode &other) const { return distance > other.distance; }
};

class World {
  public:
    World(); // default bleachers & potential field ready

    /** Add default bleachers. */
    void reset();

    /** Replace bleachers with those found in EaglePacket (object_type==0). */
    void reset_from_eagle_packet(const EaglePacket &packet);

    /** (bleacher, distance) in metres; (undefined, 1e9f) if no bleacher left. */
    [[nodiscard]] std::pair<Bleacher, float> closest_bleacher(float x, float y) const;

    /* read‑only access for planners / visualisation */
    [[nodiscard]] const auto &potential_ready() const { return potential_field_[ready_field_]; }
    [[nodiscard]] const auto &bleacher_list() const { return bleachers_; }

  private:
    // State of the world
    SizedArray<Bleacher, 10> bleachers_;

    // Shortest path
    uint8_t ready_field_ = 1;
    std::array<std::array<float, FIELD_HEIGHT_SQ>, FIELD_WIDTH_SQ> potential_field_[2]{};

    BoundedPriorityQueue<PQueueNode, FIELD_WIDTH_SQ * FIELD_HEIGHT_SQ> pqueue_;

    [[nodiscard]] auto &potential_calculating() { return potential_field_[1 - ready_field_]; }

    void init_default_bleachers();
    void reset_dijkstra();
    void partial_compute_dijkstra();
};
