#ifndef NICOLAS_H
#define NICOLAS_H

#include "iot01A/top_driver.h"

void nicolas_top_init(config_t* config);
void nicolas_top_step(config_t* config, input_t* input, output_t* output);

#endif