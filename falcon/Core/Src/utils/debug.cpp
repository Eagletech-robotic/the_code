#include "utils/debug.hpp"

#include <cfloat>
#include <cmath>
#include <cstdio>

#include "eaglesteward/constants.hpp"
#include "utils/angles.hpp"
#include "utils/myprintf.hpp"

void visualize_potential_field(std::array<std::array<float, FIELD_HEIGHT_SQ>, FIELD_WIDTH_SQ> potential_field,
                               const int width, const int height) {
    int const colors[] = {
        17, 19, 20, 26, 32, 38, 46, 82, 118, 154, 190, 226, 214, 208, 202, 196, 160, 124, 88, 52,
    };

    float minValue = FLT_MAX;
    float maxValue = -FLT_MAX;

    for (int x = 0; x < width; ++x) {
        for (int y = 0; y < height; ++y) {
            if (potential_field[x][y] == FLT_MAX)
                continue;
            if (potential_field[x][y] < minValue)
                minValue = potential_field[x][y];
            if (potential_field[x][y] > maxValue)
                maxValue = potential_field[x][y];
        }
    }

    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            if (potential_field[x][y] == FLT_MAX) {
                printf("\033[48;5;0m  \033[0m"); // Print black for obstacles
                continue;
            }
            float range = maxValue - minValue;
            if (range == 0)
                range = FLT_MIN;

            const float normalized = (potential_field[x][y] - minValue) / range;
            const int index = std::floor(normalized * (sizeof(colors) / sizeof(colors[0]) - 1));

            printf("\033[48;5;%dm  \033[0m", colors[index]);
        }
        printf("\n");
    }

    printf("Min: %f, Max: %f\n", minValue, maxValue);
}

void print_complete_input(const input_t &input) {
    host_printf(
        "Input: jack_removed:%d tof_m:%.3f delta_yaw:%.3f delta_encoder_left:%d delta_encoder_right:%d "
        "imu_yaw:%.3f imu_accel_x_mss:%.3f imu_accel_y_mss:%.3f imu_accel_z_mss:%.3f blue_button:%d clock_ms:%u",
        input.jack_removed, input.tof_m, to_degrees(input.delta_yaw),
        static_cast<int>(input.delta_encoder_left),  // STM32 passes int32_t as long int
        static_cast<int>(input.delta_encoder_right), // STM32 passes int32_t as long int
        to_degrees(input.imu_yaw), input.imu_accel_x_mss, input.imu_accel_y_mss, input.imu_accel_z_mss,
        input.blue_button,
        static_cast<unsigned int>(input.clock_ms)); // STM32 passes uint32_t as long unsigned int
}

void print_complete_output(const output_t &output) {
    host_printf("Output left:%.3f right:%.3f shovel:%.3f led:%.3f", output.motor_left_ratio, output.motor_right_ratio,
                output.shovel_ratio, output.led_ratio);
}
