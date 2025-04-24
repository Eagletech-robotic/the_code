/*
 * cc_root.cpp
 *
 *  Created on: Apr 24, 2025
 *      Author: nboulay
 */

#include "iot01A/top_driver.h"
#include "eaglesteward/behaviortree.hpp"
#include "eaglesteward/cc_retour.hpp"
#include "eaglesteward/state.hpp"
#include "math.h"
#include "utils/myprintf.hpp"
// --- Comportement racine définit ici.

float length(float x1, float y1, float x2, float y2) {
	return sqrtf((x1 - x2)*(x1 - x2) + (y1 - y2)*(y1 - y2));
}

Status isSafe(input_t * input, output_t *output, state_t *state){
	float d = length(state->x_m, state->y_m, state->opponent_x_m, state->opponent_y_m);

	if (input->tof_m < 0.1) {
		return Status::FAILURE;
	}

	if (d < 0.3 && input->tof_m < 0.2) {
		return Status::FAILURE;
	}

	if (d < 0.5) {
		return Status::FAILURE;
	}

	return Status::SUCCESS;
}

Status avoidOpponent(input_t * input, output_t *output, state_t *state) {
	myprintf("goto 90° de l'adversaire du coté target\n");

	return Status::RUNNING;
}

//TODO
Status haveBleacher(input_t * input, output_t *output, state_t *state) {
	myprintf("est-ce que j'ai un gradin accrocher ? Camera ou pelle out ou contacteur");
	return Status::FAILURE;
}
Status gotoClosestArea(input_t * input, output_t *output, state_t *state) {
	myprintf("Approcher d'une zone et lacher le gradin");
	return Status::RUNNING;
}

//TODO
Status gotoClosestBleacher(input_t * input, output_t *output, state_t *state) {
	myprintf("Approcher d'un gradin et l'attraper");
	return Status::RUNNING;
}

// Arbre de haut niveau
Status cc_root_behavior_tree(input_t * input, output_t *output, state_t *state) {
	auto safe = alternative(isSafe,avoidOpponent); // Trop proche de l'adversaire, il faut se dérouter
	auto find = alternative(haveBleacher,gotoClosestBleacher);
	auto deposit = alternative(gotoClosestBleacher);
	auto root = sequence(safe, retour, find, deposit);

	return root(input, output, state);
}
