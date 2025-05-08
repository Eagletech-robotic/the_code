#include "eaglesteward/guidance/behavior.hpp"
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
int isNearSpaceFree(State *state) { return state->filtered_tof_m > 0.5f; }

// Le robot ou une bordure ou un grand gradin sont proches (<15cm)
// La chose est détecté aussi avec un gradin au contact
int isBigThingClose(State *state) { return state->filtered_tof_m < 0.26f; }

// On a ces chiffres si le gradin est là mais aussi si on approche
int isBleacherPossiblyAtContact(State *state) { return isInRange(0.32f, state->filtered_tof_m, 0.49); }

// On passe par le mini vers 0.3 puis cela remonte.
int isPossiblyBleacherApproch(State *state) { return isInRange(0.28f, state->filtered_tof_m, 0.5); }

int isPossiblyBleacherApprochMinimum(State *state) {
    return state->filtered_tof_m < 0.32f;
} // ~15cm, ceux minimum peut descendre à 0.27 mais pas toujours

// Machine d'état qui suit l'approche d'un gradin et en déduit que l'on est au contact
// L'idée est de la faire tourner tout le temps
// si on est à BG_GET_IT, c'est qu'il faut déployer la pelle pour attraper un bleacher
// Trop sensible pour être utilisable
void fsm_getbleacher(State *state) {
    switch (state->bleacher_state) {
    case BleacherState::RESET: {
        myprintf("BleacherState::RESET\n");
        if (!isNearSpaceFree(state)) {
            state->bleacher_state = BleacherState::NON_FREE;
            return;
        }
    }; break;
    case BleacherState::NON_FREE:
        myprintf("BleacherState::NON_FREE\n");
        if (isNearSpaceFree(state)) {
            state->bleacher_state = BleacherState::RESET;
            return;
        }
        if (isPossiblyBleacherApprochMinimum(state)) {
            state->bleacher_state = BleacherState::CLOSE;
            return;
        };
        break;
    case BleacherState::CLOSE:
        myprintf("BleacherState::CLOSE");
        if (isNearSpaceFree(state)) {
            state->bleacher_state = BleacherState::RESET;
            return;
        }
        if (isBleacherPossiblyAtContact(state)) {
            state->bleacher_state = BleacherState::GET_IT;
            return;
        };
        break;
    case BleacherState::GET_IT:
        myprintf("BleacherState::GET_IT");
        if (!isBleacherPossiblyAtContact(state)) {
            state->bleacher_state = BleacherState::RESET;
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
Status evaluateSafety(input_t *input, Command *command, State *state) {
    // float x, y, theta_deg;
    // state->getPositionAndOrientation(x, y, theta_deg);

    if (state->filtered_tof_m < 0.2) {
        // failsafe si tout à merder avant
        myprintf("Failsafe\n");
        return Status::FAILURE;
    } else if (isBigThingClose(state)) {
        // -> sensible à la detection autour de la table
        myprintf("BigThing\n");
        return Status::FAILURE;
        // } else if (isBigThingClose(state) && !robot_border_outward(3.0f, 2.0f, 0.3f, x, y, theta_deg, 0.01f)) {
        //     return Status::FAILURE;
    } else {
        return Status::SUCCESS;
    }
}

// Trop proche de l'adversaire, il faut se dérouter
Status evadeOpponent(input_t *input, Command *command, State *state) {
    myprintf("goto 90° de l'adversaire du coté target\n");
    command->target_left_speed = 0.5;
    command->target_right_speed = -0.5;
    return Status::RUNNING;
}

// Détection d'un gradin accrocher au aimant ?
Status hasBleacherAttached(input_t *input, Command *command, State *state) {
    myprintf("Est-ce que j'ai un gradin accroché ? Camera ou pelle out ou TOF");

    return isBleacherPossiblyAtContact(state) ? Status::SUCCESS //-> la pelle doit avoir été sorti avant
                                              : Status::FAILURE;
}

Status goToClosestBuildingArea(input_t *input, Command *command, State *state) {
    myprintf("Aller vers une aire de construction et lâcher le gradin");
    return Status::RUNNING;
}

// TODO
Status gotoClosestBleacher(input_t *input, Command *command, State *state) {
    // world.getShortestPathToBleacher(robotx, roboty, obstacles, &state.current_path)
    // gradient_descent(state.current_path, command)
    myprintf("Aller vers un gradin et l'attraper");
    return Status::RUNNING;
}

Status detectJackRemoval(input_t *input, Command *command, State *state) {
    if (!state->hasGameStarted() && input->jack_removed) {
        state->startGame(input->clock_ms);
    }
    myprintf("T %f\n", state->elapsedTime(*input));
    return input->jack_removed ? Status::SUCCESS : Status::FAILURE;
}

Status isGameActive(input_t *input, Command *command, State *state) {
    return state->elapsedTime(*input) < 90.0f ? Status::SUCCESS : Status::FAILURE;
}

Status isStagingPhaseActive(input_t *input, Command *command, State *state) {
    return state->elapsedTime(*input) > 75.0f ? Status::FAILURE : Status::SUCCESS;
}

Status goToStaging(input_t *input, Command *command, State *state) {
    myprintf("Aller vers la zone d'attente du backstage\n");
    return Status::RUNNING;
}

Status isBackstagePhaseActive(input_t *input, Command *command, State *state) {
    return state->elapsedTime(*input) > 95.0f ? Status::FAILURE : Status::SUCCESS;
}

Status goToBackstage(input_t *input, Command *command, State *state) {
    myprintf("Aller en backstage\n");
    return Status::RUNNING;
}

// Attente indéfinie
Status waiting(input_t *input, Command *command, State *state) {
    myprintf("Waiting\n");
    command->target_left_speed = 0.f;
    command->target_right_speed = 0.f;
    command->shovel = ShovelCommand::SHOVEL_RETRACTED;
    return Status::RUNNING;
}

// For use in an alternative node, thus returning Status::FAILURE
auto logAndFail(char const *s) {
    return [s](input_t *input, Command *command, State *state) -> Status {
        myprintf("%s\n", s);
        return Status::FAILURE;
    };
}

// DEBUG - used by infiniteRectangle
Status gotoTarget(float target_imu_x, float target_imu_y, int target_nb, input_t *input, Command *command,
                  State *state) {
    if (state->target_nb != target_nb) {
        return Status::SUCCESS;
    }

    myprintf("B%d\r\n", state->target_nb);
    const bool hasArrived =
        controller_pid(state->imu_x, state->imu_y, state->imu_theta_deg, target_imu_x, target_imu_y, 0.8f, WHEELBASE_M,
                       0.08, &command->target_left_speed, &command->target_right_speed);
    if (hasArrived) {
        state->target_nb++;
        return Status::SUCCESS;
    } else {
        return Status::RUNNING;
    }
}

// DEBUG - move around a rectangle
Status infiniteRectangle(const input_t *input, Command *command, State *state) {
    auto seq = sequence(
        //
        [](input_t *input_, Command *command_, State *state_) {
            return gotoTarget(0.6, 0.0, 0, input_, command_, state_);
        },
        [](input_t *input_, Command *command_, State *state_) {
            return gotoTarget(0.6, 0.6, 1, input_, command_, state_);
        },
        [](input_t *input_, Command *command_, State *state_) {
            return gotoTarget(0.0, 0.6, 2, input_, command_, state_);
        },
        [](input_t *input_, Command *command_, State *state_) {
            return gotoTarget(0.0, 0.0, 3, input_, command_, state_);
        },
        [](input_t *, Command *, State *state_) {
            state_->target_nb = 0;
            return Status::SUCCESS;
        });

    return seq(const_cast<input_t *>(input), command, state);
}

// Top level node
Status cc_root_behavior_tree(const input_t *input, Command *command, State *state) {
    fsm_getbleacher(state);

    auto waitingStart = alternative(detectJackRemoval, logAndFail("waiting-to-start"), waiting);
    auto ended = alternative(isGameActive, logAndFail("game-ended"), waiting);
    auto safety = alternative(evaluateSafety, logAndFail("evade-opponent"), evadeOpponent);
    auto staging = alternative(isStagingPhaseActive, logAndFail("go-to-staging"), goToStaging);
    auto backstage = alternative(isBackstagePhaseActive, logAndFail("go-to-backstage"), goToBackstage);
    auto findBleacher = alternative(hasBleacherAttached, logAndFail("find-bleacher"), gotoClosestBleacher);
    auto dropBleacher = alternative(goToClosestBuildingArea, logAndFail("drop-bleacher"));
    auto root =
        sequence(waitingStart, ended, safety, infiniteRectangle, backstage, staging, findBleacher, dropBleacher);

    return root(const_cast<input_t *>(input), command, state);
}
