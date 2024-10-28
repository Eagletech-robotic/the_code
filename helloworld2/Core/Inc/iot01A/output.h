#pragma once

typedef struct output_t {
	float ratio2;
	float ratio15;
	float servo_pelle_ratio; // 0 à 1.0
} output_t;

void outputSet(output_t *output);
void outputInit(output_t * output);

