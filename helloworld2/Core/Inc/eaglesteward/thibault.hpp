#pragma once

#include "iot01A/config.h"
#include "iot01A/input.h"
#include "iot01A/output.h"

#ifdef __cplusplus
extern "C" {
#endif

void thibault_top_init(config_t* config);
void thibault_top_step(config_t* config, input_t* input, output_t* output);

#ifdef __cplusplus
}
#endif
