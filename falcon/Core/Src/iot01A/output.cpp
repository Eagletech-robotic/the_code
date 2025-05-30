
// fonction de génération des sorties.
//  le but est de pouvoir tester les autres fonctions indépendement de la cible
//  et de générer les sorties de façon fixe temporrellement à chaque cycle

#include "iot01A/output.h"

#include <stdio.h>

#include "iot01A/led.h"
#include "iot01A/motor.h"
#include "iot01A/pwm.h"
#include "utils/myprintf.hpp"

void output_init(output_t &output) {
    output.motor_left_ratio = 0.0f;
    output.motor_right_ratio = 0.0f;
    output.shovel_ratio = 0.0f; // 0.05 -> 1ms 0.1 -> 2ms, 0 off
}

void output_set(const output_t &output) {
    if (output.motor_left_ratio < -1.0 || 1.0 < output.motor_left_ratio) {
        myprintf("V2 survitesse (%f)\r\n", output.motor_left_ratio);
    }
    if (output.motor_right_ratio < -1.0 || 1.0 < output.motor_right_ratio) {
        myprintf("V1 survitesse (%f)\r\n", output.motor_right_ratio);
    }
    motorSet(output.motor_right_ratio, output.motor_left_ratio);

    PWMSet_16(output.shovel_ratio);
    led_1(output.led_ratio);
}

void print_output(const output_t &output) {
    myprintf("O %.2f %.2f %.2f\n", output.motor_left_ratio, output.motor_right_ratio, output.shovel_ratio);
}
