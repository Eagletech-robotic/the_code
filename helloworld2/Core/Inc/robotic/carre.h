/*
 * carre.h
 *
 *  Created on: Aug 30, 2024
 *      Author: nboulay
 */

#pragma once
#include "iot01A/output.h" //TODO : to remove
typedef struct carre_t {
    // output_t output;
    float time_from_start_s;     // free running compteur depuis le début
    float start_sequence_time_s; // compteur remis à la date de début de la séquence
    float timer_period_s;
    float mag[3];

} carre_t;

void carre_in_loop(carre_t *c, output_t *output);
void carre_init(carre_t *carre, float cycle_period_s);
void carre_in_loop_with_mag(carre_t *c, const float M[3], output_t *output);
void carre_in_loop_with_heading(carre_t *c, const float heading, output_t *output);
int carre_test();
