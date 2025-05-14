#pragma once

#include "iot01A/input.h"
#include "iot01A/output.h"

#include "robotic/constants.hpp"

#include <array>

// Function to visualize a potential field using colored output in the terminal
// Parameters:
// - potential_field: 2D array of floats representing the potential field
// - width: Width of the field
// - height: Height of the field
void visualize_potential_field(std::array<std::array<float, FIELD_HEIGHT_SQ>, FIELD_WIDTH_SQ>, int width, int height);

void print_complete_input(const input_t &input);
void print_complete_output(const output_t &output);
