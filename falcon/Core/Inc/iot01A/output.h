#pragma once

#ifdef SIMULATOR
#include "eaglesteward/constants.hpp"
#include <array>
#endif

// !!!!!! IMPORTANT !!!!!!
// Changes to this structure must be reflected in the simulator.
typedef struct output_t {
    float motor_left_ratio;  // ratio15
    float motor_right_ratio; // ratio2
    float shovel_ratio;      // 0.05 -> 1ms 0.1 -> 2ms, 0 off
    float led_ratio;         // définit la luminosité

    // Simulator only
#ifdef SIMULATOR
    std::array<std::array<float, FIELD_HEIGHT_SQ>, FIELD_WIDTH_SQ> potential_field{};
#endif
} output_t;

void output_set(const output_t &output);
void output_init(output_t &output);
void print_output(const output_t &output);
