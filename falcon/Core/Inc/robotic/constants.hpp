#pragma once
#include <math.h>

// Robot physical constants
static constexpr float WHEELBASE_M = 0.33f; // Distance between the wheels
static constexpr float WHEEL_CIRCUMFERENCE_M = .069f * M_PI;
static constexpr int TICKS_PER_REV = 72000; // Encoder ticks per revolution of the wheel

// Potential field for gradient descent
static constexpr int SQUARE_SIZE_CM = 4;
static constexpr float SQUARE_SIZE_M = SQUARE_SIZE_CM / 100.0f;

// Field dimensions
static constexpr int FIELD_WIDTH_SQ = 300 / SQUARE_SIZE_CM;
static constexpr int FIELD_HEIGHT_SQ = 200 / SQUARE_SIZE_CM;
static constexpr float FIELD_WIDTH_M = FIELD_WIDTH_SQ * SQUARE_SIZE_M;
static constexpr float FIELD_HEIGHT_M = FIELD_HEIGHT_SQ * SQUARE_SIZE_M;

// Robot default position and orientation on the field.
static constexpr float INITIAL_ORIENTATION = M_PI_2;
static constexpr float INITIAL_X = 1.225f;
static constexpr float INITIAL_Y = 0.225f;

// Bleacher grabbing
static constexpr float BLEACHER_WAYPOINT_DISTANCE = 0.4f;      // Distance to bleacher's centre on an orthogonal axis
static constexpr float BLEACHER_ATTRACTION_HALF_LENGTH = 0.5f; // Length of the attraction area along orthogonal axis
static constexpr float BLEACHER_ATTRACTION_HALF_WIDTH = 0.15f; // Width of the attraction area along bleacher axis
