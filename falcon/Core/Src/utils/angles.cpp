#include "utils/angles.hpp"
#include <cmath>

/**
 * Return the angle in degrees normalized to the range [-180, 180).
 */
float angle_normalize_deg(float angle_deg) {
    angle_deg = fmodf(angle_deg, 360.0f);
    if (angle_deg >= 180.0f) {
        angle_deg -= 360.0f;
    } else if (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

/**
 * Return the angle in radians normalized to the range [-PI, PI).
 */
float angle_normalize(float angle) {
    angle = fmodf(angle, 2.0f * M_PI);
    if (angle >= M_PI) {
        angle -= 2.0f * M_PI;
    } else if (angle < -M_PI) {
        angle += 2.0f * M_PI;
    }
    return angle;
}
