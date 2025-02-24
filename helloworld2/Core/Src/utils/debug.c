
#include "utils/debug.h"

#include <float.h>
#include <stdio.h>
#include <string.h>

#include "utils/constants.h"

void visualize_potential_field(float potential_field[P_FIELD_W][P_FIELD_H], size_t width,
                               size_t height) {
    int colors[] = {
        17, 19, 20, 26, 32, 38, 46, 82, 118, 154, 190, 226, 214, 208, 202, 196, 160, 124, 88, 52,
    };

    float minValue = FLT_MAX;
    float maxValue = -FLT_MAX;

    for (size_t x = 0; x < width; ++x) {
        for (size_t y = 0; y < height; ++y) {
            if (potential_field[x][y] < minValue) minValue = potential_field[x][y];
            if (potential_field[x][y] > maxValue) maxValue = potential_field[x][y];
        }
    }

    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            float normalized = (potential_field[x][y] - minValue) / (maxValue - minValue);
            int index = (int)(normalized * ((sizeof(colors) / sizeof(colors[0])) - 1));

            printf("\033[48;5;%dm  \033[0m", colors[index]);
        }
        printf("\n");
    }

    printf("Min: %f, Max: %f\n", minValue, maxValue);
}