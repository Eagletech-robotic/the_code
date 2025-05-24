#pragma once
#include <array>
#include <cfloat>

#include "utils/myprintf.hpp"

struct RollingHistory {
    static constexpr int SIZE = 250;
    std::array<float, SIZE> deltas_x{}, deltas_y{}, deltas_theta{};
    int idx{0};

    void clear();

    void push(float delta_x, float delta_y, float delta_theta);

    /**
     * Integrate the last steps and return the cumulated relative position.
     */
    void integrate_last_steps(int nb_steps, float &out_relative_x, float &out_relative_y,
                              float &out_relative_theta) const;

    /**
     * Find the nearest pose in the history that matches the given relative position.
     */
    void find_nearest_pose(float relative_x, float relative_y, int max_steps, int &out_nb_steps) const;

    /**
     * Calculate the rotation amplitude in the rectangle defined by relative coordinates.
     */
    void rotation_span_in_area(float min_x, float min_y, float max_x, float max_y, int max_steps,
                               float &out_rotation_span) const;

    void print_for_debug() const;
};
