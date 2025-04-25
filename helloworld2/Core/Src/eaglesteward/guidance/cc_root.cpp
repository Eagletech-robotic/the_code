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
#include "eaglesteward/pelle.hpp"

// --- Comportement racine définit ici.
// ce qui commence par is/has est seulement un test qui retourne Failure ou success
// le reste est une action qui retourne normalement SUCCESS ou RUNNING
// ce n'est pas obligatoire, juste un mini convention pour s'y retrouver

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

// Détection d'un gradin accorcher au aimant ?
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

// gestion du jack et du temps
Status isJackGone(input_t * input, output_t *output, state_t *state) {
    // Départ
    if(!state->previous_is_jack_gone) {
    	if(input->is_jack_gone) {
    		// Start !
    		state->start_time_ms = input->ms;
    	}
    }
    state->previous_is_jack_gone = input->is_jack_gone;
    state->elapsed_time_s = (input->ms - state->start_time_ms) / 1000.0f;

    if(input->is_jack_gone) {
    	return Status::SUCCESS;
    }
    return Status::FAILURE;
}

Status isGameEnding(input_t * input, output_t *output, state_t *state) {
	if(state->elapsed_time_s > 90.0f) {
		return Status::SUCCESS;
	 }
	return Status::FAILURE;
}

Status isNotTimeToGoToBAckstage(input_t * input, output_t *output, state_t *state) {
	if(state->elapsed_time_s > 70.0f) {
		return Status::FAILURE;
	 }
	return Status::SUCCESS;
}

Status gotoBackstage(input_t * input, output_t *output, state_t *state) {
	myprintf("Fini on rentre en backsage\n");
	return Status::RUNNING;
}

// Attente indéfinit
Status waiting(input_t * input, output_t *output, state_t *state) {
	myprintf("Waiting\n");
	output->motor_right_ratio = 0.0f;
	output->motor_left_ratio = 0.0f;
	pelle_in(output);
	return Status::RUNNING;
}

// Arbre de haut niveau
Status cc_root_behavior_tree(input_t * input, output_t *output, state_t *state) {
	auto start = alternative(isJackGone,waiting);
	auto ending = alternative(isGameEnding, waiting);
	auto backstage = alternative(isNotTimeToGoToBAckstage, gotoBackstage);
	auto safe = alternative(isSafe,avoidOpponent); // Trop proche de l'adversaire, il faut se dérouter
	auto find = alternative(haveBleacher,gotoClosestBleacher);
	auto deposit = alternative(gotoClosestBleacher);
	auto root = sequence(start, ending, backstage, safe, retour, find, deposit);

	return root(input, output, state);
}
