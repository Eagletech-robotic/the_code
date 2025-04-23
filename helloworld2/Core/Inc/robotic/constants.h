#pragma once

#include <math.h>

// Il s'agit de constantes physiques du robot qui n'ont pas de sens à être modifier (sauf erreur) en mise au point pour
// une même machine

#define WHEELBASE_M 0.33f                    // entraxe, distance entre les centres des roues ou des chenilles.
#define WHEEL_CIRCUMFERENCE_M (.069f * M_PI) // périmètre de la roue (ou roue menante si c’est un char à chenilles).
#define TICKS_PER_REV 72000                  // nombre d’impulsion d'odo par tour de roue
