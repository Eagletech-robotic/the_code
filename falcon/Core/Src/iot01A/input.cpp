/*
 * input.c
 *
 *  Created on: Nov 1, 2024
 *      Author: nboulay
 */

#include "iot01A/input.h"

#include <inttypes.h>
#include <math.h>

#include "iot01A/async_uart.h"
#include "iot01A/encoder.h"
#include "iot01A/sensors.h"
#include "iot01A/um7.hpp"
#include "utils/angles.hpp"
#include "utils/myprintf.hpp"

extern TIM_HandleTypeDef htim1;  //
extern TIM_HandleTypeDef htim2;  // PWM1 et PWM2
extern TIM_HandleTypeDef htim3;  // encoder 1
extern TIM_HandleTypeDef htim5;  // encoder 2
extern TIM_HandleTypeDef htim15; // pwm sur la led
extern TIM_HandleTypeDef htim16; // pwm16 servo
extern TIM_HandleTypeDef htim17; // pwm17 servo

void input_init(input_t &input) {
    startToF();
    encoder_init(&htim5);
    encoder_init(&htim3);
    async_uart_init();
}

int64_t encoder_raw[2];
int64_t encoder_old[2];
float yaw_raw, yaw_old;
int first_cycle = 100;

// un compteur compte en 32 bit unsigned et cela fait des mauvaises surprises
int32_t diff_with_overflow(int64_t new_, int64_t old, int64_t max) {
    int64_t r = new_ - old;
    if (r > max / 2) {
        r -= max;
    } else if (r < -max / 2) {
        r += max;
    }
    return r;
}

bool jack_removed() {
    GPIO_PinState pinState = HAL_GPIO_ReadPin(JACK_GPIO_Port, JACK_Pin);

    return pinState == GPIO_PIN_SET;
}

void input_get(input_t &input) {
    encoder_old[0] = encoder_raw[0];
    encoder_old[1] = encoder_raw[1];
    encoder_raw[0] = encoder_get_value(&htim3);
    encoder_raw[1] = encoder_get_value(&htim5);
    yaw_old = yaw_raw;
    int dist_mm;

    getDistance(&dist_mm);

    input.delta_encoder_left = -diff_with_overflow(encoder_raw[1], encoder_old[1], 4294967295);
    input.delta_encoder_right = diff_with_overflow(encoder_raw[0], encoder_old[0], 65535);
    input.tof_m = dist_mm / 1000.0;
    input.jack_removed = jack_removed();
    um7_t um7;
    um7_get_pos(&um7);
    yaw_raw = to_radians(-um7.yaw); // sign du yaw inversé
    if (first_cycle) {              // sinon il y a toujours le décalage avec old à zero
        yaw_old = yaw_raw;
        first_cycle--; // plusieurs cycle pour attendre les données
    }
    input.delta_yaw = angle_normalize(yaw_raw - yaw_old);
    input.imu_yaw = yaw_raw;
    input.imu_accel_x_mss = um7.accel_x;
    input.imu_accel_y_mss = um7.accel_y;
    input.imu_accel_z_mss = um7.accel_z;
    input.blue_button = !HAL_GPIO_ReadPin(BLUE_BUTTON_GPIO_Port, BLUE_BUTTON_Pin);
    input.clock_ms = HAL_GetTick(); // gestion du temps
}

void print_input(const input_t &input) {
    myprintf("IN %6ld %ld %ld [%d %d] %.3f\r\n", input.clock_ms, (int32_t)input.delta_encoder_left,
             (int32_t)input.delta_encoder_right, input.jack_removed, input.blue_button, input.tof_m);
}
