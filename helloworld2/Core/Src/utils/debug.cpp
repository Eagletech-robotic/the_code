#include "utils/debug.hpp"

#include <cfloat>
#include <cmath>
#include <cstdio>

#include "utils/constants.hpp"
#include "utils/myprintf.hpp"

void visualize_potential_field(float potential_field[FIELD_WIDTH_SQ][FIELD_HEIGHT_SQ], const size_t width,
                               const size_t height) {
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

void print_complete_input(const input_t &input) {
    myprintf(
        "Input: is_jack_gone:%d tof_m:%.3f delta_yaw_deg:%.3f delta_encoder_left:%d delta_encoder_right:%d "
        "imu_yaw_deg:%.3f imu_accel_x_mss:%.3f imu_accel_y_mss:%.3f imu_accel_z_mss:%.3f blue_button:%d clock_ms:%u",
        input.is_jack_gone, input.tof_m, input.delta_yaw_deg, input.delta_encoder_left, input.delta_encoder_right,
        input.imu_yaw_deg, input.imu_accel_x_mss, input.imu_accel_y_mss, input.imu_accel_z_mss, input.blue_button,
        input.clock_ms);
}

void print_complete_output(const output_t &output) {
    myprintf("Output: motor_left_ratio:%.3f motor_right_ratio:%.3f servo_pelle_ratio:%.3f servo_extra_ratio:%.3f "
             "led_ratio:%.3f",
             output.motor_left_ratio, output.motor_right_ratio, output.servo_pelle_ratio, output.servo_extra_ratio,
             output.led_ratio);
}
