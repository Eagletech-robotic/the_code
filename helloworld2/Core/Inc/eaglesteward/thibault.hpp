#pragma once

#include "iot01A/top_driver.h"
#include "eaglesteward/state.h"

#ifdef __cplusplus
extern "C" {
#endif

void thibault_top_init(config_t* config);
void thibault_top_step_bridge(input_t* input, const state_t* state, output_t* output);
void thibault_top_step(input_t* input, const state_t* state, output_t* output);

#ifdef __cplusplus
}
#endif
