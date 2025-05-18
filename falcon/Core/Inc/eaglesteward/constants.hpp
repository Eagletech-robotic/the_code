#pragma once
#include <math.h>

// Constants for the 2025 Coupe de France game

// Potential field for gradient descent
static constexpr int SQUARE_SIZE_CM = 4;
static constexpr float SQUARE_SIZE_M = SQUARE_SIZE_CM / 100.0f;

// Field dimensions
static constexpr int FIELD_WIDTH_SQ = 300 / SQUARE_SIZE_CM;
static constexpr int FIELD_HEIGHT_SQ = 200 / SQUARE_SIZE_CM;
static constexpr float FIELD_WIDTH_M = 3.0;
static constexpr float FIELD_HEIGHT_M = 2.0;

// Robot default position and orientation on the field.
static constexpr float INITIAL_ORIENTATION = M_PI_2;
static constexpr float INITIAL_X = 1.775f;
static constexpr float INITIAL_Y = 0.225f;

// Bleacher pickup
static constexpr float BLEACHER_WAYPOINT_DISTANCE = 0.4f;      // Distance to bleacher's centre on an orthogonal axis
static constexpr float BLEACHER_ATTRACTION_HALF_LENGTH = 0.5f; // Length of the attraction area along orthogonal axis
static constexpr float BLEACHER_ATTRACTION_HALF_WIDTH = 0.15f; // Width of the attraction area along bleacher axis

// Building area
static constexpr float BUILDING_AREA_WAYPOINT_DISTANCE = 0.4f; // Distance to the edge oriented toward the field
static constexpr float BUILDING_AREA_LENGTH_SMALL = 0.15f;
static constexpr float BUILDING_AREA_LENGTH_LARGE = 0.45f;
static constexpr float BUILDING_AREA_WIDTH = 0.45f;
static constexpr float BUILDING_AREA_ATTRACTION_HALF_LENGTH =
    0.5f; // Length of the attraction area along orthogonal axis
static constexpr float BUILDING_AREA_ATTRACTION_HALF_WIDTH = 0.15f; // Width of the attraction area along bleacher axis
