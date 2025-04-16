#pragma once

typedef struct output_t {
	float vitesse1_ratio; //ratio2
	float vitesse2_ratio; //ratio15
	float servo_pelle_ratio; // 0.05 -> 1ms 0.1 -> 2ms, 0 off
	float servo_en_plus;
	float led_ratio; // définit la luminosité
} output_t;

void output_set(output_t *output);
void output_init(output_t * output);

void output_print(output_t * output);
