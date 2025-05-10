#include "eaglesteward/behavior.hpp"
#include "eaglesteward/behaviortree.hpp"
#include "eaglesteward/robot_constants.hpp"
#include "eaglesteward/state.hpp"
#include "eaglesteward/tof.hpp"
#include "math.h"
#include "robotic/angle.hpp"
#include "robotic/controller_stanley.hpp"
#include "utils/myprintf.hpp"

void descend(Command &command, State &state) {
    constexpr float MAX_SPEED = 1.0f; // m/s
    auto &world = state.world;

    float x, y, orientation_deg;
    state.getPositionAndOrientation(x, y, orientation_deg);
    myprintf("Position: x=%.3f y=%.3f angle=%.3f\n", x, y, orientation_deg);

    bool is_moving;
    float target_angle_deg;
    world.potential_field_descent(x, y, is_moving, target_angle_deg);

    if (is_moving) {
        float const angle_diff = angle_normalize_deg(target_angle_deg - orientation_deg);

        if (std::abs(angle_diff) >= 90) {
            if (angle_diff >= 0) {
                command.target_left_speed = 0.0f;
                command.target_right_speed = 0.5f;
            } else {
                command.target_left_speed = 0.5f;
                command.target_right_speed = 0.0f;
            }
        } else {
            float const speed_left = 0.5f - angle_diff / 180.0f;
            float const speed_right = 0.5f + angle_diff / 180.0f;
            float const max = std::max(speed_left, speed_right);
            command.target_left_speed = MAX_SPEED / max * speed_left;
            command.target_right_speed = MAX_SPEED / max * speed_right;
        }
    } else {
        command.target_left_speed = 0.f;
        command.target_right_speed = 0.f;
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
bool isLookingOutwards(float w, float h, float s, float x, float y, float theta_deg, float tol) {
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

// On n'utilise pas la présence du robot adverse, pour être robuste sur ce sujet
Status isSafe(input_t *input, Command *command, State *state) {
    float x, y, theta_deg;
    state->getPositionAndOrientation(x, y, theta_deg);

    if (state->filtered_tof_m < 0.2) {
        // failsafe si tout à merder avant
        myprintf("Failsafe\n");
        return Status::FAILURE;
    }

    if (isBigThingClose(*state) && !isLookingOutwards(3.0f, 2.0f, 0.3f, x, y, theta_deg, 0.01f)) {
        myprintf("BigThing\n");
        return Status::FAILURE;
    }

    return Status::SUCCESS;
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

    return isBleacherPossiblyAtContact(*state) ? Status::SUCCESS //-> la pelle doit avoir été sorti avant
                                               : Status::FAILURE;
}

Status goToClosestBuildingArea(input_t *input, Command *command, State *state) {
    myprintf("Aller vers une aire de construction et lâcher le gradin");
    state->world.set_target(TargetType::BuildingAreaWaypoint);
    descend(*command, *state);

    return Status::RUNNING;
}

Status gotoClosestBleacher(input_t *input, Command *command, State *state) {
    auto seq = sequence(
        //
        [](input_t *input_, Command *command_, State *state_) {
            float x, y, orientation_deg;
            state_->getPositionAndOrientation(x, y, orientation_deg);
            auto closest_bleacher = state_->world.closest_bleacher(x, y);
            if (closest_bleacher.second > 0.35f) {
                myprintf("Go to the nearest bleacher\n");
                state_->world.set_target(TargetType::BleacherWaypoint);
                descend(*command_, *state_);
                return Status::RUNNING;
            } else {
                return Status::SUCCESS;
            }
        },
        [](input_t *input_, Command *command_, State *state_) {
            float x, y, orientation_deg;
            state_->getPositionAndOrientation(x, y, orientation_deg);
            auto closest_bleacher = state_->world.closest_bleacher(x, y);

            float const delta_x = x - closest_bleacher.first.x;
            float const delta_y = y - closest_bleacher.first.y;
            float const target_angle_deg = std::atan2(-delta_y, -delta_x) / static_cast<float>(M_PI) * 180;
            float const angle_diff = angle_normalize_deg(target_angle_deg - orientation_deg);

            if (angle_diff >= 10) {
                if (angle_diff >= 0) {
                    command_->target_left_speed = 0.5f;
                    command_->target_right_speed = -0.5f;
                } else {
                    command_->target_left_speed = -0.5f;
                    command_->target_right_speed = 0.5f;
                }

                myprintf("Rotating towards the bleacher - target angle: %f - angle diff: %f\n", target_angle_deg,
                         angle_diff);
                return Status::RUNNING;
            } else {
                return Status::SUCCESS;
            }
        },
        [](input_t *input_, Command *command_, State *state_) {
            float x, y, orientation_deg;
            state_->getPositionAndOrientation(x, y, orientation_deg);
            auto closest_bleacher = state_->world.closest_bleacher(x, y);
            if (closest_bleacher.second > 0.15f) {
                command_->target_left_speed = 0.5f;
                command_->target_right_speed = 0.5f;
                return Status::RUNNING;
            } else {
                return Status::SUCCESS;
            }
        },
        [](input_t *input_, Command *command_, State *state_) {
            myprintf("Bleacher is close enough, stop moving\n");
            command_->shovel = ShovelCommand::SHOVEL_EXTENDED;
            command_->target_left_speed = 0.f;
            command_->target_right_speed = 0.f;
            return Status::RUNNING;
        });

    return seq(const_cast<input_t *>(input), command, state);
}

Status isJackRemoved(input_t *input, Command *command, State *state) {
    if (!state->hasGameStarted() && input->jack_removed) {
        state->startGame(input->clock_ms);
    }

    if (input->jack_removed) {
    	return Status::SUCCESS;
    }
    state->GameNotStarted();
    return Status::FAILURE;
}

Status isGameActive(input_t *input, Command *command, State *state) {
    return state->elapsedTime(*input) < 90.0f ? Status::SUCCESS : Status::FAILURE;
}

Status isBackstagePhaseNotActive(input_t *input, Command *command, State *state) {
    return state->elapsedTime(*input) > 80.0f ? Status::FAILURE : Status::SUCCESS;
}

Status goToBackstage(input_t *input, Command *command, State *state) {
    myprintf("Aller en backstage\n");
    state->world.set_target(TargetType::BackstageWaypoint);
    descend(*command, *state);

    return Status::RUNNING;
}

// Attente indéfinie
Status wait(input_t *input, Command *command, State *state) {
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


Status is2secondGone(const input_t *input, Command *command, State *state) {
	if( state->elapsedTime(*input) > 2.0f) {
		return Status::SUCCESS;
	}
	return Status::FAILURE;
}

Status forward(const input_t *input, Command *command, State *state) {
	command->target_left_speed = 0.7f;
	command->target_right_speed = 0.7f;
	return Status::RUNNING;

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
Status top_behavior(const input_t *input, Command *command, State *state) {
    updateTofStateMachine(*state);

    auto root = sequence( //
        alternative(isJackRemoved, logAndFail("wait-start"), wait),
        alternative(isGameActive, logAndFail("hold-after-stop"), wait),
        alternative(isSafe, logAndFail("ensure-safety"), evadeOpponent),
		alternative(is2secondGone,logAndFail("forward_2sec"), forward),
		alternative(logAndFail("STOP"),wait),
        alternative(isBackstagePhaseNotActive, logAndFail("go-to-backstage"), goToBackstage),
        alternative(hasBleacherAttached, logAndFail("grab-bleacher"), gotoClosestBleacher),
        alternative(logAndFail("drop-bleacher"), goToClosestBuildingArea));
    return root(const_cast<input_t *>(input), command, state);
}
