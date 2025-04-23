#ifndef VISUALIZE_POTENTIAL_FIELD_H
#define VISUALIZE_POTENTIAL_FIELD_H

#include <cstddef> // For size_t

#include "utils/constants.hpp"

// Function to visualize a potential field using colored output in the terminal
// Parameters:
// - potential_field: 2D array of floats representing the potential field
// - width: Width of the field
// - height: Height of the field
void visualize_potential_field(float potential_field[FIELD_WIDTH_SQ][FIELD_HEIGHT_SQ], size_t width, size_t height);

#endif // VISUALIZE_POTENTIAL_FIELD_H
