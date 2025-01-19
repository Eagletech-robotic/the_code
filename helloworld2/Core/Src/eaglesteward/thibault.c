#include "iot01A/top_driver.h"

void thibault_top_init(config_t* config) {}

void thibault_top_step(config_t* config, input_t* input, output_t* output) {
    output->vitesse1_ratio = 1.0;
    output->vitesse2_ratio = 0.5;
}
