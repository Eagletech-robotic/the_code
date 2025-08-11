#pragma once

#include "iot01A/config.h"
#include "iot01A/input.h"
#include "iot01A/output.h"

// fonction executé par top.c dans le robot et par le server http dans le simulateur
// il ne doit pas y avoir de dépendance avec les spécificités ARM ou ST

void top_init(config_t &config);
void top_step(const config_t &config, const input_t &input, output_t &output);
