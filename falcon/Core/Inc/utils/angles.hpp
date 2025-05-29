#pragma once

#include <math.h>

constexpr float RAD_TO_DEG = 180.0f / static_cast<float>(M_PI);
constexpr float DEG_TO_RAD = static_cast<float>(M_PI) / 180.0f;

inline float to_degrees(float radians) { return radians * RAD_TO_DEG; }
inline float to_radians(float degrees) { return degrees * DEG_TO_RAD; }

float angle_normalize_deg(float angle_deg);
float angle_normalize(float angle);
bool isLookingOutwards(float w, float h, float s, float x, float y, float theta, float tol);
bool isBInFrontOfA(float ax, float ay, float aTheta, float bx, float by);
