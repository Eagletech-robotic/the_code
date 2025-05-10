#include <emscripten/emscripten.h>
#include <stdio.h>
#include <stdlib.h>

#include "iot01A/top_driver.h"
#include "robotic/bluetooth.hpp"

config_t config;

extern "C" {
EMSCRIPTEN_KEEPALIVE void exported_top_init() { top_init(config); }

EMSCRIPTEN_KEEPALIVE input_t *create_input() {
    input_t *ptr = (input_t *) malloc(sizeof(input_t));
    if (!ptr) {
        printf("Error: Failed to allocate input_t\n");
        return NULL;
    }
    return ptr;
}

EMSCRIPTEN_KEEPALIVE output_t *create_output() {
    output_t *ptr = (output_t *) malloc(sizeof(output_t));
    if (!ptr) {
        printf("Error: Failed to allocate output_t\n");
        return NULL;
    }
    return ptr;
}

int constexpr BLUETOOTH_BLOCK_SIZE = 100;

EMSCRIPTEN_KEEPALIVE uint8_t *create_bluetooth() {
    uint8_t *ptr = (uint8_t *) malloc(sizeof(uint8_t) * BLUETOOTH_BLOCK_SIZE);
    if (!ptr) {
        printf("Error: Failed to allocate bluetooth\n");
        return NULL;
    }
    return ptr;
}

EMSCRIPTEN_KEEPALIVE void exported_top_step(input_t *input, output_t *output, uint8_t *bluetooth_block,
                                            size_t bluetooth_block_size) {
    if (!input || !output) {
        printf("Error: NULL input or output in exported_top_step\n");
        return;
    }

    if (bluetooth_block != NULL) {
        for (size_t i = 0; i < bluetooth_block_size; i++) {
            g_bluetooth_decoder.byte_received(bluetooth_block[i]);
        }
    }

    top_step(config, *input, *output);
}
} // extern "C"

int main() { return 0; }
