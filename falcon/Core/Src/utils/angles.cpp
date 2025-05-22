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

/*
 * Retourne true si le robot :
 *   • est entièrement contenu dans la carte (w × h),
 *   • regarde vers l’extérieur (aligné sur +X, +Y, –X ou –Y, dans la tolérance),
 *   • et s’il reste moins d’un côté libre (s) entre sa face avant et la bordure
 *     — donc impossible d’insérer un obstacle carré de même taille devant lui.
 *
 * Tous les paramètres sont en float.
 *   w, h : dimensions de la carte
 *   s    : côté du robot
 *   x, y : position du centre du robot
 *   theta : orientation du robot
 *   tol  : marge sur les comparaisons de distances
 *
 * La tolérance angulaire d’alignement est fixée à ±30° (modifiable).
 */
bool isLookingOutwards(float w, float h, float s, float x, float y, float theta, float tol) {
    const float theta_deg = to_degrees(theta);

    /* Directions cardinales en degrés                     +X  +Y  –X   –Y */
    const float dirs_deg[4] = {0.0f, 90.0f, 180.0f, 270.0f};
    const float ALIGN_TOL = 30.0f; /* ±30° d’ouverture                 */

    int dir = -1;
    for (int i = 0; i < 4; ++i) {
        /* écart minimal, en tenant compte du wrap-around 0°/360° */
        float delta = fabsf(theta_deg - dirs_deg[i]);
        delta = fminf(delta, 360.0f - delta);
        if (delta < ALIGN_TOL) {
            dir = i;
            break;
        }
    }
    if (dir == -1) /* pas suffisamment aligné            */
        return false;

    /* ─── 2. robot entièrement dans la carte ? ─────────────────────────── */
    float r = s * 0.5f; /* demi-côté                          */
    if (x < r - tol || x > w - r + tol || y < r - tol || y > h - r + tol)
        return false; /* déborde (au moins partiellement)   */

    /* ─── 3. distance face avant ↔ bord associé ────────────────────────── */
    float dist;
    switch (dir) {
    case 0:
        dist = w - (x + r);
        break; /* → +X  (bord droit)       */
    case 1:
        dist = h - (y + r);
        break; /* ↑ +Y  (bord haut)        */
    case 2:
        dist = x - r;
        break; /* ← –X  (bord gauche)      */
    case 3:
        dist = y - r;
        break; /* ↓ –Y  (bord bas)         */
    default:
        return false; /* ne devrait jamais arriver*/
    }

    /* ─── 4. verdict : place libre < côté du robot ? ───────────────────── */
    return dist < s - tol;
}