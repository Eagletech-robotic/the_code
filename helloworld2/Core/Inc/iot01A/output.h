#pragma once

typedef struct output_t {
	float vitesse1_ratio; //ratio2
	float vitesse2_ratio; //ratio15
	float servo_pelle_ratio; // 0 à 1.0
} output_t;

void output_set(output_t *output);
void output_init(output_t * output);

void output_print(output_t * output);
