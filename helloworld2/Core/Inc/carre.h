/*
 * carre.h
 *
 *  Created on: Aug 30, 2024
 *      Author: nboulay
 */

#pragma once
#include "output.h"
typedef struct carre_t {
	output_t output;
	float time_from_start_s; //free running compteur depuis le début
	float start_sequence_time_s;  // compteur remis à la date de début de la séquence
	float timer_period_s;
} carre_t;

void carre_in_loop(carre_t * carre);
void carre_init(carre_t * carre,float cycle_period_s) ;

int carre_test();
