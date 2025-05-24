#pragma once
#include <math.h>

// Robot physical constants
static constexpr float WHEELBASE_M = 0.33f; // Distance between the wheels
static constexpr float WHEEL_CIRCUMFERENCE_M = .069f * M_PI;
static constexpr int TICKS_PER_REV = 72000;     // Encoder ticks per revolution of the wheel
static constexpr float ROBOT_RADIUS = 0.19f;    // Radius of the robot
static constexpr float SHOVEL_TO_CENTER = 0.10; // Distance between the shovel and the center of the robot
static constexpr float SPEED_MAX = 2.0f;        // Maximum speed of the robot

// Potential field for gradient descent
static constexpr int SQUARE_SIZE_CM = 4;
static constexpr float SQUARE_SIZE_M = SQUARE_SIZE_CM / 100.0f;

// Field dimensions
static constexpr int FIELD_WIDTH_SQ = 300 / SQUARE_SIZE_CM;
static constexpr int FIELD_HEIGHT_SQ = 200 / SQUARE_SIZE_CM;
static constexpr float FIELD_WIDTH_M = 3.00;
static constexpr float FIELD_HEIGHT_M = 2.00;

// Robot default position and orientation on the field.
static constexpr float INITIAL_ORIENTATION = M_PI_2;
static constexpr float INITIAL_X = 3 - 1.775f;
static constexpr float INITIAL_Y = 0.15f;

// Bleacher
static constexpr float BLEACHER_WIDTH = 0.10f;
static constexpr float BLEACHER_WAYPOINT_DISTANCE = 0.30f;      // Distance to bleacher's centre on an orthogonal axis
static constexpr float BLEACHER_ATTRACTION_HALF_LENGTH = 0.40f; // Length of the attraction area along orthogonal axis
static constexpr float BLEACHER_ATTRACTION_HALF_WIDTH = 0.15f;  // Width of the attraction area along bleacher axis

// Building area
static constexpr float BUILDING_AREA_WAYPOINT_DISTANCE = 0.30f; // Distance to the edge oriented toward the field
static constexpr float BUILDING_AREA_LENGTH_SMALL = 0.15f;
static constexpr float BUILDING_AREA_LENGTH_LARGE = 0.45f;
static constexpr float BUILDING_AREA_WIDTH = 0.45f;
static constexpr float BUILDING_AREA_ATTRACTION_HALF_LENGTH =
    0.50f; // Length of the attraction area along orthogonal axis
static constexpr float BUILDING_AREA_ATTRACTION_HALF_WIDTH = 0.15f; // Width of the attraction area along bleacher axis
