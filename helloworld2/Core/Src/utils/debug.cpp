#include "utils/debug.hpp"

#include <cfloat>
#include <cmath>
#include <cstdio>

#include "utils/constants.hpp"

void visualize_potential_field(float potential_field[FIELD_WIDTH_SQ][FIELD_HEIGHT_SQ], const size_t width, const size_t height) {
    int const colors[] = {
        17, 19, 20, 26, 32, 38, 46, 82, 118, 154, 190, 226, 214, 208, 202, 196, 160, 124, 88, 52,
    };

    float minValue = FLT_MAX;
    float maxValue = -FLT_MAX;

    for (size_t x = 0; x < width; ++x) {
        for (size_t y = 0; y < height; ++y) {
            if (potential_field[x][y] < minValue)
                minValue = potential_field[x][y];
            if (potential_field[x][y] > maxValue)
                maxValue = potential_field[x][y];
        }
    }

    for (size_t y = 0; y < height; ++y) {
        for (size_t x = 0; x < width; ++x) {
            float const normalized = (potential_field[x][y] - minValue) / (maxValue - minValue);
            int const index = std::floor(normalized * (sizeof(colors) / sizeof(colors[0]) - 1));

            printf("\033[48;5;%dm  \033[0m", colors[index]);
        }
        printf("\n");
    }

    // for (size_t y = 0; y < height; ++y) {
    //     for (size_t x = 0; x < width; ++x) {
    //         printf("%1.0f,", potential_field[x][y]);
    //     }
    //     printf("\n");
    // }

    printf("Min: %f, Max: %f\n", minValue, maxValue);
}
