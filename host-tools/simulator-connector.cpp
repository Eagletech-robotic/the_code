#include <emscripten/emscripten.h>
#include <stdio.h>
#include <stdlib.h>

#include "iot01A/top_driver.h"

config_t config;

extern "C" {

EMSCRIPTEN_KEEPALIVE void exported_top_init() { top_init(&config); }

EMSCRIPTEN_KEEPALIVE input_t *create_input() {
    input_t *ptr = (input_t *)malloc(sizeof(input_t));
    if (!ptr) {
        printf("Error: Failed to allocate input_t\n");
        return NULL;
    }
    return ptr;
}

EMSCRIPTEN_KEEPALIVE output_t *create_output() {
    output_t *ptr = (output_t *)malloc(sizeof(output_t));
    if (!ptr) {
        printf("Error: Failed to allocate output_t\n");
        return NULL;
    }
    return ptr;
}

EMSCRIPTEN_KEEPALIVE void exported_top_step(input_t *input, output_t *output) {
    if (!input || !output) {
        printf("Error: NULL input or output in exported_top_step\n");
        return;
    }

    top_step(&config, input, output);
}

int main() { return 0; }

} // extern "C"
