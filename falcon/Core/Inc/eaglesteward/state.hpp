#pragma once

#include "eaglesteward/world.hpp"
#include "iot01A/config.h"
#include "iot01A/input.h"
#include "robotic/pid.hpp"
#include "rolling_history.hpp"

enum class TofState { CLEAR_PATH, OBJECT_DETECTED, OBJECT_NEARBY, BLEACHER_CONTACT };

class State {
  public:
    State() { reset(); }

    void reset();

    /* I/O ----------------------------------------------------- */
    void print() const;

    [[nodiscard]] bool hasGameStarted() const;

    void gameNotStarted();

    void startGame(uint32_t clock_ms);

    [[nodiscard]] float elapsedTime(const input_t &input) const;

    void updateFromInput(const config_t &cfg, const input_t &in);

    void updateFromBluetooth(float elapsed_time);

    /* PUBLIC DATA ---------------------------------------------------- */
    RobotColour colour{RobotColour::Yellow};

    // TODO
    // 1 - Should we overwrite the IMU coordinates with the field coordinates when we receive a packet?
    // 2 - We should store the IMU movements for the last n milliseconds, where n is the average time from picture to
    // packet reception. Alternatively, discard the camera position if the robot has moved too much in the last n
    // milliseconds.

    // Field coordinates.
    float robot_x{0.f}; // meters
    float robot_y{0.f};
    float robot_theta{0.f};
    float last_robot_update_time{-FLT_MAX}; // Elapsed time of the last position update from the camera

    // World
    World world{colour};

    // TOF
    float filtered_tof_m{1.f};
    TofState tof_state{TofState::CLEAR_PATH};

    // Motors
    PID_t pid_diff{};
    PID_t pid_sum{};

    // Navigation
    PID_t pid_theta{};
    PID_t pid_speed{};
    RollingHistory odo_history{};
    bool is_moving_forward;

    float on_evade_since{0.0f};

    // Bluetooth
    bool packet_received_at_this_step{false};

    // LED
    int32_t led_lighted_at_ms{-1}; // Time in clock_ms when the LED was last turned on. -1 means off.

    // TimeTelemetry
    // TODO: add time telemetries for step time and gradient descent (or whatever) time.
    // class TimeTelemetry {
    //   constructor(name);
    //   void tick(step_nb, time_ms);
    //   void dump(); -> "{name}: [379]0.008 [386]0.007 [AVG] 0.003"
    // };

    // Pseudo state
    std::uint32_t bt_tick = 0;

    // Bleacher carrying
    bool bleacher_lifted{false};

    // Target coordinates for PID approach
    GameEntity target{0.0f, 0.0f, 0.0f};

    // --- Method ---

    [[nodiscard]] bool is_target_set() const { return target.x != 0.f || target.y != 0.f; }

    void lock_target(float x, float y, float orientation = 0.f) {
        target.x = x;
        target.y = y;
        target.orientation = orientation;
    }

    void release_target() {
        target.x = 0.f;
        target.y = 0.f;
        target.orientation = 0.f;
    }

    /* END PUBLIC DATA ------------------------------------ */

  private:
    /* PRIVATE DATA ---------------------------------- */
    // Time management
    int32_t start_time_ms{-1}; // The input's clock_ms at the start of the game. -1 means game has not started.
    /* END PRIVATE DATA --------------------- */
};
