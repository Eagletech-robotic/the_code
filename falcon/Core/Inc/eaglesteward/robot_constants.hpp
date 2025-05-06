#pragma once

#include <math.h>

// Il s'agit de constantes physiques du robot qui n'ont pas de sens à être modifier (sauf erreur) en mise au point pour
// une même machine

static constexpr float WHEELBASE_M = 0.33f;                  // Distance entre les centres des roues.
static constexpr float WHEEL_CIRCUMFERENCE_M = .069f * M_PI; // Périmètre de la roue.
static constexpr int TICKS_PER_REV = 72000;                  // Nombre d’impulsion d'odo par tour de roue
