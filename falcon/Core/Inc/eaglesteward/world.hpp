#pragma once

#include "eaglesteward/game_entities.hpp"
#include "robotic/eagle_packet.hpp"
#include "utils/bounded_pqueue.hpp"
#include "utils/sized_array.hpp"

#include <array>
#include <utility>

struct PQueueNode {
    float distance;
    uint8_t x;
    uint8_t y;

    bool operator<(const PQueueNode &other) const { return distance > other.distance; }
};

enum class ObstacleType {
    None,    // Keep the order: each status has prevalence over previous ones
    Movable, // Movable > None
    Fixed    // Fixed > Movable
};

enum class TargetType {
    None,
    BleacherWaypoint,
    BackstageWaypoint,
    BuildingAreaWaypoint,
    TestPoint0, // pour test de la descente
    TestPoint1,
    TestPoint2,
    TestPoint3,
};

class World {
  public:
    explicit World(RobotColour colour);

    /** Set the next target for the robot. */
    void set_target(TargetType target);

    /** Replace objects with those found in EaglePacket. */
    void update_from_eagle_packet(const EaglePacket &packet);

    /** Return the yaw angle of the steepest slope in the potential field, from the robot's position. */
    bool potential_field_descent(float x, float y, float arrival_distance, float &out_yaw) const;

    /** Do some calculations that fit in a step. Returns true if calculations were done. */
    bool do_some_calculations(const std::function<bool()> &can_continue);

    /** Do all calculations. Used to pre-compute the potential field. */
    void do_all_calculations_LONG();

    /** Return the closest bleacher to the given coordinates. */
    [[nodiscard]] std::pair<Bleacher *, float> closest_available_bleacher(float x, float y);

    void remove_bleacher(float x, float y);

    [[nodiscard]] BuildingArea *closest_building_area(float x, float y, bool only_available);

    [[nodiscard]] const auto &potential_ready() const { return potential_field_[ready_field_]; }

    RobotColour colour_;

    // Opponent robot
    float opponent_x{0.f}; // meters
    float opponent_y{0.f};
    float opponent_theta{0.f};

    // Default bleachers positions
    SizedArray<Bleacher, 10> default_bleachers_;

    // State of the world
    SizedArray<Bleacher, 10> bleachers_;
    SizedArray<Can, 40> cans_;
    SizedArray<Plank, 20> planks_;
    SizedArray<BuildingArea, 8> building_areas_;

    // Potential field
    TargetType target_ = TargetType::None; // Leave None, so that the field is re-computed the first time it changes
    uint8_t ready_field_ = 1;
    std::array<std::array<float, FIELD_HEIGHT_SQ>, FIELD_WIDTH_SQ> potential_field_[2]{};

    std::array<std::array<ObstacleType, FIELD_HEIGHT_SQ>, FIELD_WIDTH_SQ> obstacles_field_{};

    BoundedPriorityQueue<PQueueNode, 5'000> pqueue_; // No upper limit, but should be enough for the field size

    [[nodiscard]] auto &potential_calculating() { return potential_field_[1 - ready_field_]; }

    void reset_dijkstra();

    bool partial_compute_dijkstra(const std::function<bool()> &can_continue);

    static bool is_in_field(float x, float y);

    static bool is_in_field_square(int i, int j);

  private:
    [[nodiscard]] float finite_potential(int i, int j) const;

    [[nodiscard]] float potential_at(float x, float y) const;

    [[nodiscard]] std::pair<float, float> bilinear_gradient(float px, float py) const;

    void enqueue_targets();

    void setup_obstacles_field();
};
