#pragma once

#include "iot01A/config.h"
#include "iot01A/input.h"
#include "iot01A/output.h"

void nicolas_top_init(config_t &config);
void nicolas_top_step(const config_t &config, const input_t &input, output_t &output);
