#pragma once

typedef struct output_t {
    float motor_left_ratio;  // ratio15
    float motor_right_ratio; // ratio2
    float servo_pelle_ratio; // 0.05 -> 1ms 0.1 -> 2ms, 0 off
    float servo_extra_ratio;
    float led_ratio; // définit la luminosité
} output_t;

void output_set(output_t *output);
void output_init(output_t *output);

void print_output(output_t *output);
