/*
 * cc_root.cpp
 *
 *  Created on: Apr 24, 2025
 *      Author: nboulay
 */

#include "eaglesteward/guidance/cc_root.hpp"
#include "eaglesteward/behaviortree.hpp"
#include "eaglesteward/robot_constants.hpp"
#include "eaglesteward/state.hpp"
#include "iot01A/top_driver.h"
#include "math.h"
#include "robotic/controller_stanley.hpp"
#include "utils/myprintf.hpp"

// --- Comportement racine définit ici.
// ce qui commence par is/has est seulement un test qui retourne Failure ou success
// le reste est une action qui retourne normalement SUCCESS ou RUNNING
// ce n'est pas obligatoire, juste un mini convention pour s'y retrouver

float length(float x1, float y1, float x2, float y2) { return sqrtf((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2)); }

// --- TOF
// Avec la position physique du 20250426
//   > 0.48 il n'y a rien
//   [0.47; 0.36], il y a un gradin collé
//    [0.48; 0.26 mini ; 0.36/0.47] en approche de gradin
//  <0.26 -> un robot approche

int isInRange(float min_, float val, float max_) { return min_ < val && val < max_; }

// A autour de 0.5 je détecte le sol à ~ 40 cm
int isNearSpaceFree(state_t *state) { return state->filtered_tof_m > 0.5f; }

// Le robot ou une bordure ou un grand gradin sont proches (<15cm)
// La chose est détecté aussi avec un gradin au contact
int isBigThingClose(state_t *state) { return state->filtered_tof_m < 0.26f; }

// On a ces chiffres si le gradin est là mais aussi si on approche
int isBleacherPossiblyAtContact(state_t *state) { return isInRange(0.32f, state->filtered_tof_m, 0.49); }

// On passe par le mini vers 0.3 puis cela remonte.
int isPossiblyBleacherApproch(state_t *state) { return isInRange(0.28f, state->filtered_tof_m, 0.5); }

int isPossiblyBleacherApprochMinimum(state_t *state) {
    return state->filtered_tof_m < 0.32f;
} // ~15cm, ceux minimum peut descendre à 0.27 mais pas toujours

// Machine d'état qui suit l'approche d'un gradin et en déduit que l'on est au contact
// L'idée est de la faire tourner tout le temps
// si on est à BG_GET_IT, c'est qu'il faut déployer la pelle pour attraper un bleacher
// Trop sensible pour être utilisable
void fsm_getbleacher(state_t *state) {
    switch (state->getbleacher_state) {
    case GB_RESET: {
        myprintf("GB_RESET\n");
        if (!isNearSpaceFree(state)) {
            state->getbleacher_state = GB_NON_FREE;
            return;
        }
    }; break;
    case GB_NON_FREE:
        myprintf("GB_NON_FREE\n");
        if (isNearSpaceFree(state)) {
            state->getbleacher_state = GB_RESET;
            return;
        }
        if (isPossiblyBleacherApprochMinimum(state)) {
            state->getbleacher_state = GB_CLOSE;
            return;
        };
        break;
    case GB_CLOSE:
        myprintf("GB_CLOSE");
        if (isNearSpaceFree(state)) {
            state->getbleacher_state = GB_RESET;
            return;
        }
        if (isBleacherPossiblyAtContact(state)) {
            state->getbleacher_state = GB_GET_IT;
            return;
        };
        break;
    case GB_GET_IT:
        myprintf("GB_GET_IT");
        if (!isBleacherPossiblyAtContact(state)) {
            state->getbleacher_state = GB_RESET;
            return;
        };
        break;
    }
}
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
bool robot_border_outward(float w, float h, float s, float x, float y, float theta_deg, float tol) {
    /* Directions cardinales en degrés                     +X  +Y  –X   –Y */
    const float dirs_deg[4] = {0.0f, 90.0f, 180.0f, 270.0f};
    const float ALIGN_TOL = 30.0f; /* ±30° d’ouverture                 */

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
    if (dir == -1) /* pas suffisamment aligné            */
        return false;

    /* ─── 2. robot entièrement dans la carte ? ─────────────────────────── */
    float r = s * 0.5f; /* demi-côté                          */
    if (x < r - tol || x > w - r + tol || y < r - tol || y > h - r + tol)
        return false; /* déborde (au moins partiellement)   */

    /* ─── 3. distance face avant ↔ bord associé ────────────────────────── */
    float dist;
    switch (dir) {
    case 0:
        dist = w - (x + r);
        break; /* → +X  (bord droit)       */
    case 1:
        dist = h - (y + r);
        break; /* ↑ +Y  (bord haut)        */
    case 2:
        dist = x - r;
        break; /* ← –X  (bord gauche)      */
    case 3:
        dist = y - r;
        break; /* ↓ –Y  (bord bas)         */
    default:
        return false; /* ne devrait jamais arriver*/
    }

    /* ─── 4. verdict : place libre < côté du robot ? ───────────────────── */
    return dist < s - tol;
}

// --- Comportement

// On n'utilise pas la présence du robot adverse, pour être robuste sur ce sujet
Status isSafe(input_t *input, Command *command, state_t *state) {
    if (state->filtered_tof_m < 0.2) { // failsafe si tout à merder avant
        myprintf("Failsafe\n");
        return Status::FAILURE;
    }

    if (isBigThingClose(state)) { // -> sensible à la detection autour de la table
        myprintf("BigThing\n");
        return Status::FAILURE;
    }

    //	if (isBigThingClose(state) && !robot_border_outward(3.0f, 2.0f, 0.3f, state->x_m, state->y_m, state->theta_deg,
    // 0.01f)) { 		return Status::FAILURE;
    //	}

    return Status::SUCCESS;
}

Status avoidOpponent(input_t *input, Command *command, state_t *state) {
    myprintf("goto 90° de l'adversaire du coté target\n");
    command->target_left_speed = 0.5;
    command->target_right_speed = -0.5;
    return Status::RUNNING;
}

// Détection d'un gradin accrocher au aimant ?
Status haveBleacher(input_t *input, Command *command, state_t *state) {
    myprintf("Est-ce que j'ai un gradin accroché ? Camera ou pelle out ou TOF");
    if (isBleacherPossiblyAtContact(state)) { //-> la pelle doit avoir été sorti avant
        return Status::SUCCESS;
    }
    return Status::FAILURE;
}

Status gotoClosestBuildingArea(input_t *input, Command *command, state_t *state) {
    myprintf("Aller vers une aire de construction et lâcher le gradin");
    return Status::RUNNING;
}

// TODO
Status gotoClosestBleacher(input_t *input, Command *command, state_t *state) {
    myprintf("Aller vers un gradin et l'attraper");
    return Status::RUNNING;
}

// gestion du jack et du temps
Status isJackGone(input_t *input, Command *command, state_t *state) {
    // Départ
    if (!state->previous_jack_removed) {
        if (input->jack_removed) {
            // Start !
            state->start_time_ms = input->clock_ms;
        }
    }
    state->previous_jack_removed = input->jack_removed;
    myprintf("T %f\n", state->elapsed_time_s);
    if (input->jack_removed) {
        state->elapsed_time_s = (input->clock_ms - state->start_time_ms) / 1000.0f;
        return Status::SUCCESS;
    }

    return Status::FAILURE;
}

Status isGameOn(input_t *input, Command *command, state_t *state) {
    if (state->elapsed_time_s < 90.0f) {
        return Status::SUCCESS;
    }
    return Status::FAILURE;
}

Status isNotTimeToGoToBackstageStaging(input_t *input, Command *command, state_t *state) {
    if (state->elapsed_time_s > 75.0f) {
        return Status::FAILURE;
    }
    return Status::SUCCESS;
}

Status isNotTimeToGoToBackstage(input_t *input, Command *command, state_t *state) {
    if (state->elapsed_time_s > 95.0f) {
        return Status::FAILURE;
    }
    return Status::SUCCESS;
}

Status goToBackstageStaging(input_t *input, Command *command, state_t *state) {
    myprintf("Aller vers la zone d'attente du backstage\n");
    return Status::RUNNING;
}

Status gotoBackstage(input_t *input, Command *command, state_t *state) {
    myprintf("Aller en backstage\n");
    return Status::RUNNING;
}

// Attente indéfinie
Status waiting(input_t *input, Command *command, state_t *state) {
    myprintf("Waiting\n");
    command->specialCommand = SpecialCommand::IMMEDIATE_STOP;
    command->shovel = ShovelCommand::SHOVEL_RETRACT;
    return Status::RUNNING;
}

// -- Debug

Status gotoTarget(float start_x_m, float start_y_m, float target_x_m, float target_y_m, float next_x_m, float next_y_m,
                  int target, input_t *input, Command *command, state_t *func_state) {
    if (func_state->target != target) {
        return Status::SUCCESS;
    }
    myprintf("B%d\r\n", func_state->target);
    int isArrived = controller_pid(func_state->x_m, func_state->y_m, func_state->theta_deg, target_x_m, target_y_m,
                                   0.8f, WHEELBASE_M, 0.08, &command->target_left_speed, &command->target_right_speed);
    if (isArrived) {
        func_state->target++;
        return Status::SUCCESS;
    }
    return Status::RUNNING;
}

// c'est prévu pour être utilisé dans une clause alternaltive, d'ou le Failure en retour
auto print(char const *s) {
    return [s](input_t *input, Command *command, state_t *state) -> Status {
        myprintf("%s\n", s);
        return Status::FAILURE;
    };
}

// execution une fois par cycle de tout l'arbre
Status cc_infinite_rectangle(const input_t *input, Command *command, state_t *func_state) {
    auto seq = sequence(
        [](input_t *lambda_input, Command *lambda_command, state_t *state) {
            return gotoTarget(0.0, 0.0, 0.6, 0.0, 0.6, 0.6, 0, lambda_input, lambda_command, state);
        },
        [](input_t *lambda_input, Command *lambda_command, state_t *state) {
            return gotoTarget(0.6, 0.0, 0.6, 0.6, 0.0, 0.6, 1, lambda_input, lambda_command, state);
        },
        [](input_t *lambda_input, Command *lambda_command, state_t *state) {
            return gotoTarget(0.6, 0.6, 0.0, 0.6, 0.0, 0.0, 2, lambda_input, lambda_command, state);
        },
        [](input_t *lambda_input, Command *lambda_command, state_t *state) {
            return gotoTarget(0.0, 0.6, 0.0, 0.0, 0.6, 0.0, 3, lambda_input, lambda_command, state);
        },
        [](input_t *, Command *, state_t *state) {
            state->target = 0;
            return Status::SUCCESS;
        });

    return seq(const_cast<input_t *>(input), command, func_state);
}

// Arbre de haut niveau
Status cc_root_behavior_tree(const input_t *input, Command *command, state_t *state) {
    fsm_getbleacher(state);
    auto start = alternative(isJackGone, print("start"), waiting);
    auto ending = alternative(isGameOn, print("ending"), waiting);
    auto safe = alternative(isSafe, print("safe"), avoidOpponent); // Trop proche de l'adversaire, il faut se dérouter
    auto backstageStaging =
        alternative(isNotTimeToGoToBackstageStaging, print("backstage-staging"), goToBackstageStaging);
    auto backstage = alternative(isNotTimeToGoToBackstage, print("backstage"), gotoBackstage);
    auto findBleacher = alternative(haveBleacher, print("start"), gotoClosestBleacher);
    auto dropBleacher = alternative(gotoClosestBuildingArea, print("deposit"));
    auto root =
        sequence(start, ending, safe, cc_infinite_rectangle, backstage, backstageStaging, findBleacher, dropBleacher);

    return root(const_cast<input_t *>(input), command, state);
}
