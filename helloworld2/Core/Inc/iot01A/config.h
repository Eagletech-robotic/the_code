#pragma once
/*
 * config.h
 *
 *  Created on: Nov 1, 2024
 *      Author: nboulay
 */

// il s'agit des "valeurs magiques" du robot qui ne bougent pas pendant le match
// mais qui pourrait avoir un intéret à modifier en mise au point

typedef struct config_t {
    int time_step_ms;
} config_t;
