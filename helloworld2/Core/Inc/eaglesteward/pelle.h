/*
 * pelle.h
 *
 *  Created on: Apr 12, 2025
 *      Author: nboulay
 */

#pragma once

#include "iot01A/output.h"

#ifdef __cplusplus
extern "C" {
#endif

void pelle_off(output_t *output);
void pelle_in(output_t *output);
void pelle_out(output_t *output);

#ifdef __cplusplus
}
#endif