#pragma once
#include <math.h>

// Robot physical constants, that apply for any game with the Eagle robot.

static constexpr float WHEELBASE_M = 0.33f; // Distance between the wheels
static constexpr float WHEEL_CIRCUMFERENCE_M = .069f * M_PI;
static constexpr int TICKS_PER_REV = 72000; // Encoder ticks per revolution of the wheel

static constexpr float SHOVEL_TO_CENTER = 0.10; // Distance between the shovel and the center of the robot
