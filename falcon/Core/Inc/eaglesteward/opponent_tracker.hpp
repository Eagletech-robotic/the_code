#pragma once
#include <array>

class OpponentTracker {
  public:
    // Add a new opponent observation to the rolling buffer.
    void push(bool detected, float elapsed_time, float x = 0.0f, float y = 0.0f);

    // Returns true if the opponent has moved more than MOVEMENT_THRESHOLD
    // in either x or y direction within the stored position window.
    bool is_alive() const;

    // Get the number of consecutive non-detections from the most recent frame.
    int get_consecutive_non_detections() const;

    // Calculate the opponent's speed vector based on the last two detections.
    bool get_speed_vector(float &out_speed_x, float &out_speed_y) const;

    // Get the current position of the opponent.
    // - If last detection was successful: returns that position.
    // - If last detection failed but we have 2 detections in last 3 frames: interpolates.
    bool get_interpolated_position(float &out_x, float &out_y) const;

    // Clear the tracking history.
    void clear();

  private:
    // Number of observations to keep in the history
    static constexpr int SIZE = 80;

    // Movement threshold for considering opponent "alive" (in meters)
    static constexpr float MOVEMENT_THRESHOLD = 0.05f;

    // Time window for checking if opponent is alive (in seconds)
    static constexpr float TIME_WINDOW_FOR_ALIVE_CHECK = 10.0f;

    std::array<float, SIZE> positions_x{}, positions_y{};
    std::array<bool, SIZE> detected{};
    std::array<float, SIZE> timestamps{};

    int idx{0};
    int valid_count{0}; // Track how many observations have been stored
};