#pragma once

#include "eaglesteward/world.hpp"
#include "iot01A/config.h"
#include "iot01A/input.h"

#include <stdint.h>

#include "robotic/pid.hpp"

enum class BleacherState { RESET, NON_FREE, CLOSE, GET_IT };

class State {
  public:
    enum class Color { BLUE = 0, YELLOW = 1 };

    State();
    void init();

    /* I/O ----------------------------------------------------- */
    void print() const;
    [[nodiscard]] bool hasGameStarted() const;
    void startGame(uint32_t clock_ms);
    [[nodiscard]] float elapsedTime(const input_t &input) const;
    void getPositionAndOrientation(float &x, float &y, float &theta) const;
    void updateFromInput(const config_t &cfg, const input_t &in);
    void updateFromBluetooth();

    /* PUBLIC DATA ---------------------------------------------------- */
    Color color{Color::BLUE};

    // Our robot, in IMU coordinates. Use getPositionAndOrientation for field coordinates.
    float imu_x{0.f}; // meters
    float imu_y{0.f};
    float imu_theta_deg{0.f};

    // Opponent robot
    float opponent_x{0.f}; // meters
    float opponent_y{0.f};
    float opponent_theta_deg{0.f};

    // World
    World world{};

    // TOF
    float filtered_tof_m{0.f};

    // Motors
    PID_t pid_diff{};
    PID_t pid_sum{};

    // Bluetooth
    bool packet_received_at_this_step{false};

    // LED
    int32_t led_lighted_at_ms{-1}; // Time in clock_ms when the LED was last turned on. -1 means off.

    // Use by the behavior tree
    int target_nb{0}; // for rectangle test
    BleacherState bleacher_state{BleacherState::RESET};
    /* END PUBLIC DATA ------------------------------------ */

  private:
    /* PRIVATE DATA ---------------------------------- */
    // IMU-to-field transformation
    float transformation_x{0.f};         // X translation offset after rotation (meters)
    float transformation_y{0.f};         // Y translation offset after rotation
    float transformation_theta_deg{0.f}; // Rotation offset between IMU and field

    // Time management
    int32_t start_time_ms{-1}; // The input's clock_ms at the start of the game. -1 means game has not started.
    /* END PRIVATE DATA --------------------- */

    void saveImuToFieldTransform(float x_field, float y_field, float theta_field);
};
