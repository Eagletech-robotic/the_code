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
    void startGame(uint32_t clock_ms);
    [[nodiscard]] float elapsedTime(const input_t &input) const;
    void updateFromInput(const config_t &cfg, const input_t &in);
    void updateFromBluetooth();
    void getPositionAndOrientation(float &x, float &y, float &theta) const;

    /* PUBLIC DATA ---------------------------------------------------- */
    Color color{Color::BLUE};

    // Opponent
    float opponent_x{0.f}; // meters
    float opponent_y{0.f};
    float opponent_theta_deg{0.f};

    // TOF
    float filtered_tof_m{0.f};

    // World
    World world{};

    // Time management
    uint32_t start_time_ms{0}; // The input's clock_ms at the start of the game

    // --- Motor PID
    PID_t pid_diff{};
    PID_t pid_sum{};

    // Nicolas
    bool previous_jack_removed{false};
    int target{0}; // for rectangle test
    BleacherState bleacher_state{BleacherState::RESET};
    /* END PUBLIC DATA ------------------------------------ */

  private:
    /* PRIVATE DATA ---------------------------------- */
    // IMU coordinate system
    float imu_x{0.f}; // meters
    float imu_y{0.f};
    float imu_theta_deg{0.f};

    // IMU to field coordinate transformation
    float transformation_x{0.f};         // X translation offset after rotation (meters)
    float transformation_y{0.f};         // Y translation offset after rotation
    float transformation_theta_deg{0.f}; // Rotation offset between IMU and field
    /* END PRIVATE DATA --------------------- */

    void saveImuToFieldTransform(float x_field, float y_field, float theta_field);
};
