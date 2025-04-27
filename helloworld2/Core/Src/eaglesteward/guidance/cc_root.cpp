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

// --- TOF
// Avec la position physique du 20250426
//   > 0.48 il n'y a rien
//   [0.47; 0.36], il y a un gradin coller
//    [0.48; 0.26 mini ; 0.36/0.47] en approche de gradin
//  <0.26 -> un robot approche

int isInRange (float min_, float val, float max_) {
	return min_ < val && val < max_;
}

// A autour de 0.5 je détecte le sol à ~ 40 cm
int isNearSpaceFree(state_t *state) {
	return state->filtered_tof_m > 0.46f;
}

// Le robot ou une bordure ou un grand gradin sont proches (<15cm)
// C'est vrai aussi avec un gradin au contact
int isBigThingClose(state_t *state) {
	return state->filtered_tof_m < 0.25f;
}

// On a ses chiffres sile gradin est là mais aussi si on approche
int isBleacherPossiblyAtContact(state_t *state){
	return isInRange(0.36f, state->filtered_tof_m, 0.47);
}

// On passe par le mini vers 0.3 puis cela remonte.
int isPossiblyBleacherApproch(state_t *state){
	return isInRange(0.3f, state->filtered_tof_m, 0.48);
}

#define DEG2RAD(d) ((d) * M_PI / 180.0)


/*
 * Retourne true si le robot :
 *   • est entièrement contenu dans la carte (w × h),
 *   • regarde vers l’extérieur (aligné sur +X, +Y, –X ou –Y, dans la tolérance),
 *   • et s’il reste moins d’un côté libre (s) entre sa face avant et la bordure
 *     — donc impossible d’insérer un obstacle carré de même taille devant lui.
 *
 * Tous les paramètres sont en float.
 *   w, h : dimensions de la carte
 *   s    : côté du robot
 *   x, y : position du centre du robot
 *   theta_deg : orientation **en degrés**
 *   tol  : marge sur les comparaisons de distances
 *
 * La tolérance angulaire d’alignement est fixée à ±30° (modifiable).
 */
bool robot_border_outward(float w, float h,
                          float s,
                          float x, float y,
                          float theta_deg,
                          float tol)
{
    /* Directions cardinales en degrés                     +X  +Y  –X   –Y */
    const float dirs_deg[4] = { 0.0f, 90.0f, 180.0f, 270.0f };
    const float ALIGN_TOL   = 30.0f;   /* ±30° d’ouverture                 */

    int dir = -1;
    for (int i = 0; i < 4; ++i) {
        /* écart minimal, en tenant compte du wrap-around 0°/360° */
        float delta = fabsf(theta_deg - dirs_deg[i]);
        delta = fminf(delta, 360.0f - delta);
        if (delta < ALIGN_TOL) {
            dir = i;
            break;
        }
    }
    if (dir == -1)                 /* pas suffisamment aligné            */
        return false;

    /* ─── 2. robot entièrement dans la carte ? ─────────────────────────── */
    float r = s * 0.5f;            /* demi-côté                          */
    if (x < r - tol || x > w - r + tol ||
        y < r - tol || y > h - r + tol)
        return false;              /* déborde (au moins partiellement)   */

    /* ─── 3. distance face avant ↔ bord associé ────────────────────────── */
    float dist;
    switch (dir) {
        case 0:  dist = w - (x + r); break;   /* → +X  (bord droit)       */
        case 1:  dist = h - (y + r); break;   /* ↑ +Y  (bord haut)        */
        case 2:  dist = x - r;        break;  /* ← –X  (bord gauche)      */
        case 3:  dist = y - r;        break;  /* ↓ –Y  (bord bas)         */
        default: return false;                /* ne devrait jamais arriver*/
    }

    /* ─── 4. verdict : place libre < côté du robot ? ───────────────────── */
    return dist < s - tol;
}

// --- Comportement

// on n'utilise pas la présence du robot adverse,pour être robuste sur ce sujet
Status isSafe(input_t * input, output_t *output, state_t *state){
	if (input->tof_m < 0.2) { // failsafe si tout à merder avant
		return Status::FAILURE;
	}

	if (isBigThingClose(state) && !robot_border_outward(3.0f, 2.0f, 0.3f, state->x_m, state->y_m, state->theta_deg, 0.01f)) {
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
	if(isBleacherPossiblyAtContact(state)) {
		return Status::SUCCESS;
	}
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
	auto root = sequence(start, ending, safe, backstage,  retour, find, deposit);

	return root(input, output, state);
}
