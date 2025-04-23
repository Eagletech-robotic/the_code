#pragma once

// Quelques opérations géométrique en 3D et objet pour manipuler un compa magnétique

void projectOnPlane(const float A[3], const float M[3], float M2D[2]);

float angleBetween2D(const float A[2], const float B[2]);
float angleBetweenVectors(const float v1[3], const float v2[3]);
float computeHeading(float mag[3], float acc[3]);
void calibrate_nr(float mag[3]);
void stat_nr(float data);
