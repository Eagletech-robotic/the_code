/*
 * pelle.c
 *
 *  Created on: Apr 12, 2025
 *      Author: nboulay
 */

#include "eaglesteward/pelle.h"

void pelle_off(output_t * output) {
	output->servo_pelle_ratio = 0.0f;
}
void pelle_in(output_t * output) {
	output->servo_pelle_ratio = 0.05f;
}
void pelle_out(output_t * output) {
	output->servo_pelle_ratio = 0.12f;
}
