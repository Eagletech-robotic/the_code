#pragma once

#include "iot01A/top_driver.h"

#ifdef __cplusplus
extern "C" {
#endif

void thibault_top_init(config_t* config);
void thibault_top_step(config_t* config, const input_t* input, output_t* output);

#ifdef __cplusplus
}
#endif
