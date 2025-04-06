/*
 * constante.h
 *
 *  Created on: Apr 6, 2025
 *      Author: nboulay
 */

// Il s'agit de constantes physiques du robot qui n'ont pas de sens à être modifier (sauf erreur) en mise au point pour une même machine

#pragma once

#include <math.h>

#define WHEEL_BASE_M   0.33 // entraxe, distance entre les centres des roues ou des chenilles.
#define WHEEL_CIRCUMFERENCE_M  .065*M_PI  // périmètre exact de la roue (ou roue menante si c’est un char à chenilles).
#define TICKS_PER_REV 72000 // nombre d’impulsion d'odo par tour de roue
