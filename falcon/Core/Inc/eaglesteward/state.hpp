#pragma once

#include "eaglesteward/world.hpp"
#include "iot01A/config.h"
#include "iot01A/input.h"
#include "robotic/pid.hpp"

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
    void getPositionAndOrientation(float &x, float &y, float &theta) const;
    void updateFromInput(const config_t &cfg, const input_t &in);
    void updateFromBluetooth();

    /* PUBLIC DATA ---------------------------------------------------- */
    RobotColour colour{RobotColour::Blue};

    // TODO
    // 1 - Should we overwrite the IMU coordinates with the field coordinates when we receive a packet?
    // 2 - We should store the IMU movements for the last n milliseconds, where n is the average time from picture to
    // packet reception. Alternatively, discard the camera position if the robot has moved too much in the last n
    // milliseconds.

    // Our robot, in IMU coordinates. Use getPositionAndOrientation for field coordinates.
    float imu_x{0.f}; // meters
    float imu_y{0.f};
    float imu_theta{0.f};

    // Opponent robot
    float opponent_x{0.f}; // meters
    float opponent_y{0.f};
    float opponent_theta{0.f};

    // World
    World world{colour};

    // TOF
    float filtered_tof_m{1.f};
    TofState tof_state{TofState::CLEAR_PATH};
    bool bleacher_lifted{false};

    // Motors
    PID_t pid_diff{};
    PID_t pid_sum{};

    // Navigation
    PID_t pid_theta{};
    PID_t pid_speed{};

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

    // Use by the behavior tree
    int target_nb{0}; // for rectangle test

    /* END PUBLIC DATA ------------------------------------ */

  private:
    /* PRIVATE DATA ---------------------------------- */
    // IMU-to-field transformation
    float transformation_x{0.f};     // X translation offset after rotation (meters)
    float transformation_y{0.f};     // Y translation offset after rotation
    float transformation_theta{0.f}; // Rotation offset between IMU and field

    // Time management
    int32_t start_time_ms{-1}; // The input's clock_ms at the start of the game. -1 means game has not started.
    /* END PRIVATE DATA --------------------- */

    void saveImuToFieldTransform(float x_field, float y_field, float theta_field);
};
