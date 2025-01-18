#include <emscripten/emscripten.h>
#include <stdio.h>

#include "iot01A/top_driver.h"

config_t config;

EMSCRIPTEN_KEEPALIVE void exported_top_init() { top_init(&config); }

EMSCRIPTEN_KEEPALIVE input_t *create_input() { return (input_t *)malloc(sizeof(input_t)); }
EMSCRIPTEN_KEEPALIVE output_t *create_output() { return (output_t *)malloc(sizeof(output_t)); }
EMSCRIPTEN_KEEPALIVE void exported_top_step(input_t *input, output_t *output) {
    top_step(&config, input, output);
}

int main() { return 0; }
