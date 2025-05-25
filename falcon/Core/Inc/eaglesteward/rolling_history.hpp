#pragma once
#include <array>

class RollingHistory {
  public:
    void rectify_odometry_from_camera_position(float camera_x, float camera_y, float camera_theta, float robot_x,
                                               float robot_y, float robot_theta, float &out_corrected_x,
                                               float &out_corrected_y, float &out_corrected_theta) const;

    void clear();

    void push(float delta_x, float delta_y, float delta_theta);

  private:
    // Number of steps to keep in the history:
    // - Too small: we are going to throw away packets if the camera pose is further away in the past.
    // - Too large: we are going to under-correct by using positions of the robot unrelated to the camera shot.
    // => Keep it slightly above the average number of lookback steps.
    static constexpr int SIZE = 100;

    std::array<float, SIZE> deltas_x{}, deltas_y{}, deltas_theta{};

    int idx{0};

    /**
     * Integrate the last steps and return the cumulated relative position.
     */
    void integrate_last_steps(int nb_steps, float &out_relative_x, float &out_relative_y,
                              float &out_relative_theta) const;

    /**
     * Find the nearest pose in the history that matches the given relative position.
     */
    void find_nearest_pose(float relative_x, float relative_y, int &out_nb_steps) const;

    /**
     * Calculate the rotation amplitude in the rectangle defined by relative coordinates.
     */
    void rotation_span_in_area(float min_x, float min_y, float max_x, float max_y, float &out_rotation_span) const;

    void print_for_debug() const;
};
