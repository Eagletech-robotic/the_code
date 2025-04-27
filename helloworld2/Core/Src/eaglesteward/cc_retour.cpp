/*
 * cc_retour.cpp
 *
 *  Created on: Apr 22, 2025
 *      Author: nboulay
 */

#include "eaglesteward/cc_retour.hpp"
#include "eaglesteward/pelle.hpp"
#include "utils/myprintf.hpp"
#include <stdint.h>
// Il s'agit du comportement de retour "backstage"
// à t+85s les PMI démarrent
// à t+86s le robot doit toucher la zone d'arrivé
// à t+90s le robot s'arrète de bouger
// à t+100S les PMI  s'arrête de bouger.

// cela implique de gérer le temp, et donc le jack

Status retour(input_t *input, output_t *output, state_t *state) {
    myprintf("Retour\n");

    // Départ
    if (!state->previous_is_jack_gone) {
        if (input->is_jack_gone) {
            // Start !
            state->start_time_ms = input->clock_ms;
        }
    }
    state->previous_is_jack_gone = input->is_jack_gone;
    state->elapsed_time_s = static_cast<float>(input->clock_ms - state->start_time_ms) / 1000.0f;

    // Fin du match
    if (state->elapsed_time_s > 90.0f) {
        // FULL STOP
        output->motor_right_ratio = 0.0f;
        output->motor_left_ratio = 0.0f;
        pelle_in(output);
        return Status::RUNNING;
    }

    // GO to backstage
    if (state->elapsed_time_s > 70.0f) {
        myprintf("goto backstage\n");
        return Status::RUNNING;
    }

    return Status::SUCCESS;
}
