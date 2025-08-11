#pragma once

#include "eaglesteward/constants.hpp"
#include "utils/bounded_pqueue.hpp"
#include <array>
#include <cfloat>
#include <cmath>
#include <cstdint>
#include <functional>

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

class PotentialField {
  public:
    /** Clear the potential field to max values */
    void clear() {
        for (auto &row : potential_) {
            std::fill(row.begin(), row.end(), FLT_MAX);
        }
    }

    /** Set a cell value */
    void set_cell(int i, int j, float value) { potential_[i][j] = value; }

    /** Get a cell value */
    [[nodiscard]] float get_cell(int i, int j) const { return potential_[i][j]; }

    /** Get mutable access to field data */
    float *data() { return potential_.data()->data(); }

    /** Get the potential value at a position */
    [[nodiscard]] float potential_at(float x, float y) const;

    /** Perform gradient descent from position. Returns true if arrived (potential <= arrival_distance) */
    bool gradient_descent(float x, float y, float arrival_distance, float &out_yaw) const;

    std::pair<float, float> find_nearest_finite_potential(float x, float y) const;

    /** Get the potential field (for debugging/visualization) */
    [[nodiscard]] const auto &get_field() const { return potential_; }

    /** Compute partial Dijkstra using provided resources. Returns true if more computation needed */
    bool
    compute_dijkstra_partial(const std::array<std::array<ObstacleType, FIELD_HEIGHT_SQ>, FIELD_WIDTH_SQ> &obstacles,
                             BoundedPriorityQueue<PQueueNode, 5'000> &pqueue,
                             const std::function<bool()> &can_continue);

  private:
    std::array<std::array<float, FIELD_HEIGHT_SQ>, FIELD_WIDTH_SQ> potential_{};

    [[nodiscard]] float finite_potential(int i, int j) const;
    [[nodiscard]] std::pair<float, float> bilinear_gradient(float px, float py) const;
    [[nodiscard]] std::pair<float, float> interpolated_gradient(float px, float py) const;
};
