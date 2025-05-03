
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
    output.servo_pelle_ratio = 0.0f; // 0.05 -> 1ms 0.1 -> 2ms, 0 off
    output.servo_extra_ratio = 0.0f;
}

void output_set(const output_t &output) {
    if (output.motor_left_ratio < -1.0 || 1.0 < output.motor_left_ratio) {
        printf("V2 survitesse (%f)\r\n", output.motor_left_ratio);
    }
    if (output.motor_right_ratio < -1.0 || 1.0 < output.motor_right_ratio) {
        printf("V1 survitesse (%f)\r\n", output.motor_right_ratio);
    }
    motorSet(output.motor_right_ratio, output.motor_left_ratio);

    PWMSet_16(output.servo_pelle_ratio);
    PWMSet_17(output.servo_extra_ratio);
    led_1(output.led_ratio);
}

void print_output(const output_t &output) {
    myprintf("O %.4f\t%.4f\t%.4f\n", output.motor_left_ratio, output.motor_right_ratio, output.servo_pelle_ratio);
}
