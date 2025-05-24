#include "eaglesteward/rolling_history.hpp"

void RollingHistory::clear() {
    deltas_x.fill(0.f);
    deltas_y.fill(0.f);
    deltas_theta.fill(0.f);
    idx = 0;
}

void RollingHistory::push(float delta_x, float delta_y, float delta_theta) {
    deltas_x[idx] = delta_x;
    deltas_y[idx] = delta_y;
    deltas_theta[idx] = delta_theta;
    idx = (idx + 1) % SIZE;
}

void RollingHistory::integrate_last_steps(int nb_steps, float &out_relative_x, float &out_relative_y,
                                          float &out_relative_theta) const {
    out_relative_x = out_relative_y = out_relative_theta = 0.f;
    for (int step_nb = 0; step_nb < nb_steps; ++step_nb) {
        int idx_back = (idx - 1 - step_nb + SIZE) % SIZE;
        out_relative_x += deltas_x[idx_back];
        out_relative_y += deltas_y[idx_back];
        out_relative_theta += deltas_theta[idx_back];
    }
}

/**
 * Find the nearest pose in the history that matches the given relative position.
 */
void RollingHistory::find_nearest_pose(float relative_x, float relative_y, int max_steps, int &out_nb_steps) const {
    out_nb_steps = 0;

    float cumulative_x = 0.0f, cumulative_y = 0.0f;
    float min_d2 = FLT_MAX;

    for (int step_nb = 0; step_nb < max_steps; ++step_nb) {
        float dx = cumulative_x - relative_x;
        float dy = cumulative_y - relative_y;
        float d2 = dx * dx + dy * dy;

        if (d2 < min_d2) {
            min_d2 = d2;
            out_nb_steps = step_nb;
        }

        int idx_back = (idx - 1 - step_nb + SIZE) % SIZE;
        cumulative_x -= deltas_x[idx_back];
        cumulative_y -= deltas_y[idx_back];
    }
}

void RollingHistory::rotation_span_in_area(float min_x, float min_y, float max_x, float max_y, int max_steps,
                                           float &out_rotation_span) const {
    out_rotation_span = 0.f;

    float cumulative_x = 0.f, cumulative_y = 0.f, cumulative_theta = 0.f;
    float min_theta = FLT_MAX, max_theta = -FLT_MAX;

    for (int step_nb = 0; step_nb < max_steps; ++step_nb) {
        if (cumulative_x >= min_x && cumulative_x <= max_x && cumulative_y >= min_y && cumulative_y <= max_y) {
            if (cumulative_theta < min_theta)
                min_theta = cumulative_theta;
            if (cumulative_theta > max_theta)
                max_theta = cumulative_theta;
        }

        const int idx_back = (idx - 1 - step_nb + SIZE) % SIZE;
        cumulative_x -= deltas_x[idx_back];
        cumulative_y -= deltas_y[idx_back];
        cumulative_theta -= deltas_theta[idx_back];
    }

    if (min_theta != FLT_MAX)                      // at least one sample in area
        out_rotation_span = max_theta - min_theta; // non-negative
}

void RollingHistory::print_for_debug() const {
    float relative_x = 0.f;
    float relative_y = 0.f;
    for (int step_nb = 0; step_nb < SIZE; ++step_nb) {
        int idx_back = (idx - 1 - step_nb + SIZE) % SIZE;
        relative_x -= deltas_x[idx_back];
        relative_y -= deltas_y[idx_back];
        myprintf("%d(%.3f %.3f) ", step_nb, relative_x, relative_y);
    }
}
