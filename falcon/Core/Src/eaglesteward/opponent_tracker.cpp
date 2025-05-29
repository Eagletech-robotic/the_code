#include "eaglesteward/opponent_tracker.hpp"

#include <algorithm>
#include <cfloat>

void OpponentTracker::push(bool is_detected, float elapsed_time, float x, float y) {
    detected[idx] = is_detected;
    timestamps[idx] = elapsed_time;

    if (is_detected) {
        positions_x[idx] = x;
        positions_y[idx] = y;
    }
    // Note: positions for non-detected frames are left as-is (don't matter)

    idx = (idx + 1) % SIZE;
    if (valid_count < SIZE) {
        valid_count++;
    }
}

bool OpponentTracker::is_alive() const {
    if (valid_count == 0) {
        return false; // No observations stored yet
    }

    // Get the most recent timestamp
    int most_recent_idx = (idx - 1 + SIZE) % SIZE;
    float most_recent_time = timestamps[most_recent_idx];
    float time_threshold = most_recent_time - TIME_WINDOW_FOR_ALIVE_CHECK;

    float min_x = FLT_MAX, max_x = -FLT_MAX;
    float min_y = FLT_MAX, max_y = -FLT_MAX;
    bool found_any_detection = false;

    // Check observations within the time window
    for (int step = 0; step < valid_count; ++step) {
        int check_idx = (idx - 1 - step + SIZE) % SIZE;

        // Stop if we've gone beyond the time window
        if (timestamps[check_idx] < time_threshold) {
            break;
        }

        // Only consider detected frames within the time window
        if (detected[check_idx]) {
            float x = positions_x[check_idx];
            float y = positions_y[check_idx];

            min_x = std::min(min_x, x);
            max_x = std::max(max_x, x);
            min_y = std::min(min_y, y);
            max_y = std::max(max_y, y);
            found_any_detection = true;
        }
    }

    if (!found_any_detection) {
        return false; // No detections in the time window
    }

    // Check if movement exceeds threshold in either direction
    return (max_x - min_x) > MOVEMENT_THRESHOLD || (max_y - min_y) > MOVEMENT_THRESHOLD;
}

int OpponentTracker::get_consecutive_non_detections() const {
    if (valid_count == 0) {
        return 0; // No observations yet
    }

    // Count consecutive non-detections from most recent backwards
    int count = 0;
    for (int step = 0; step < valid_count; ++step) {
        int check_idx = (idx - 1 - step + SIZE) % SIZE;
        if (!detected[check_idx]) {
            count++;
        } else {
            break; // Found a detection, stop counting
        }
    }

    return count;
}

bool OpponentTracker::get_speed_vector(float &out_speed_x, float &out_speed_y) const {
    // Need at least 3 observations
    if (valid_count < 3) {
        return false;
    }

    // Get indices for the last 3 frames
    int frame0_idx = (idx - 1 + SIZE) % SIZE; // Most recent
    int frame1_idx = (idx - 2 + SIZE) % SIZE; // Second most recent
    int frame2_idx = (idx - 3 + SIZE) % SIZE; // Third most recent

    if (detected[frame0_idx]) {
        // Last detection was successful - calculate speed to previous detection
        if (detected[frame1_idx]) {
            // Speed from frame 1 to frame 0
            out_speed_x = positions_x[frame0_idx] - positions_x[frame1_idx];
            out_speed_y = positions_y[frame0_idx] - positions_y[frame1_idx];
            return true;
        } else if (detected[frame2_idx]) {
            // Speed from frame 2 to frame 0 (2 frame difference)
            out_speed_x = (positions_x[frame0_idx] - positions_x[frame2_idx]) / 2.0f;
            out_speed_y = (positions_y[frame0_idx] - positions_y[frame2_idx]) / 2.0f;
            return true;
        }
    } else {
        // Last detection failed - check if we can interpolate from frame 1 and 2
        if (detected[frame1_idx] && detected[frame2_idx]) {
            // Speed from frame 2 to frame 1
            out_speed_x = positions_x[frame1_idx] - positions_x[frame2_idx];
            out_speed_y = positions_y[frame1_idx] - positions_y[frame2_idx];
            return true;
        }
    }

    return false;
}

bool OpponentTracker::get_interpolated_position(float &out_x, float &out_y) const {
    // Need at least 1 observation
    if (valid_count < 1) {
        return false;
    }

    // Get most recent frame index
    int frame0_idx = (idx - 1 + SIZE) % SIZE;

    if (detected[frame0_idx]) {
        // Last detection was successful - return that position
        out_x = positions_x[frame0_idx];
        out_y = positions_y[frame0_idx];
        return true;
    }

    // Last detection failed - check if we can interpolate
    if (valid_count < 3) {
        return false;
    }
    int frame1_idx = (idx - 2 + SIZE) % SIZE; // Second most recent
    int frame2_idx = (idx - 3 + SIZE) % SIZE; // Third most recent

    if (detected[frame1_idx] && detected[frame2_idx]) {
        // Calculate speed from frame 2 to frame 1
        float speed_x = positions_x[frame1_idx] - positions_x[frame2_idx];
        float speed_y = positions_y[frame1_idx] - positions_y[frame2_idx];

        // Extrapolate 1 frame forward from frame 1
        out_x = positions_x[frame1_idx] + speed_x;
        out_y = positions_y[frame1_idx] + speed_y;
        return true;
    }

    return false;
}

void OpponentTracker::clear() {
    positions_x.fill(0.f);
    positions_y.fill(0.f);
    detected.fill(false);
    idx = 0;
    valid_count = 0;
}