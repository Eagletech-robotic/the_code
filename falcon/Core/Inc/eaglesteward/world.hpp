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

enum class TargetType {
    None,
    BleacherWaypoint,
    StagingWaypoint,
    BuildingAreaWaypoint,
};

class World {
  public:
    explicit World(RobotColour colour);

    /** Set the next target for the robot. */
    void set_target(TargetType target);

    /** Replace objects with those found in EaglePacket. */
    void update_from_eagle_packet(const EaglePacket &packet);

    /** Return the yaw angle of the steepest slope in the potential field, from the robot's position. */
    void potential_field_descent(float x, float y, float &out_speed, float &out_yaw_deg) const;

    /** Do some calculations that fit in a step. Returns true if calculations were done. */
    bool do_some_calculations();

    [[nodiscard]] const auto &potential_ready() const { return potential_field_[ready_field_]; }

  private:
    RobotColour colour_;

    // State of the world
    SizedArray<Bleacher, 10> bleachers_;

    // Potential field
    TargetType target_ = TargetType::None;
    uint8_t ready_field_ = 1;
    std::array<std::array<float, FIELD_HEIGHT_SQ>, FIELD_WIDTH_SQ> potential_field_[2]{};

    BoundedPriorityQueue<PQueueNode, FIELD_WIDTH_SQ * FIELD_HEIGHT_SQ> pqueue_;

    [[nodiscard]] auto &potential_calculating() { return potential_field_[1 - ready_field_]; }

    void reset_potential_field();
    void partial_compute_dijkstra();
};
