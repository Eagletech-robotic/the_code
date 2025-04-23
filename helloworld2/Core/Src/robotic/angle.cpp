/*
 * angle.c
 *
 *  Created on: Apr 17, 2025
 *      Author: nboulay
 */
#include "robotic/angle.hpp"
#include <cmath>

float angle_normalize_deg(float angle_deg) {
    // Remet dans [-360, 360] (fmod renvoie un résultat dans (-360, 360], sauf si angle_deg est un multiple exact de
    // 360)
    angle_deg = fmodf(angle_deg, 360.0f);

    // Maintenant, on s'assure que c'est dans [-180, 180)
    if (angle_deg >= 180.0f) {
        angle_deg -= 360.0f;
    } else if (angle_deg < -180.0f) {
        angle_deg += 360.0f;
    }
    return angle_deg;
}
