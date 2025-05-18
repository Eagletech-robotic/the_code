#include "eaglesteward/behavior.hpp"
#include "eaglesteward/behaviortree.hpp"
#include "eaglesteward/state.hpp"
#include "eaglesteward/tof.hpp"
#include "math.h"
#include "robotic/controllers.hpp"
#include "robotic/robot_constants.hpp"
#include "utils/angles.hpp"
#include "utils/myprintf.hpp"

void descend(Command &command, State &state) {
    constexpr float MAX_SPEED = 1.0f; // m/s
    auto &world = state.world;

    float x, y, orientation;
    state.getPositionAndOrientation(x, y, orientation);

    bool is_local_minimum;
    float target_angle;
    world.potential_field_descent(x, y, is_local_minimum, target_angle);

    if (is_local_minimum) {
        // Move forward slowly rather than remaining trapped
        command.target_left_speed = 0.3f;
        command.target_right_speed = 0.3f;
    } else {
        float const angle_diff = angle_normalize(target_angle - orientation);

        if (std::abs(angle_diff) >= M_PI_2) {
            if (angle_diff >= 0) {
                command.target_left_speed = -0.5f;
                command.target_right_speed = 0.5f;
            } else {
                command.target_left_speed = 0.5f;
                command.target_right_speed = -0.5f;
            }
        } else {
            float const speed_left = 0.5f - angle_diff / M_PI;
            float const speed_right = 0.5f + angle_diff / M_PI;
            float const max = std::max(speed_left, speed_right);
            command.target_left_speed = MAX_SPEED / max * speed_left;
            command.target_right_speed = MAX_SPEED / max * speed_right;
        }
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
 *   theta : orientation du robot
 *   tol  : marge sur les comparaisons de distances
 *
 * La tolérance angulaire d’alignement est fixée à ±30° (modifiable).
 */
bool isLookingOutwards(float w, float h, float s, float x, float y, float theta, float tol) {
    const float theta_deg = to_degrees(theta);

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
    float x, y, theta;
    state->getPositionAndOrientation(x, y, theta);

    if (state->filtered_tof_m < 0.2) {
        // failsafe si tout à merder avant
        myprintf("Failsafe\n");
        return Status::FAILURE;
    }

    if (isBigThingClose(*state) && !isLookingOutwards(3.0f, 2.0f, 0.3f, x, y, theta, 0.01f)) {
        myprintf("BigThing\n");
        return Status::FAILURE;
    }

    return Status::SUCCESS;
}

// Trop proche de l'adversaire, il faut se dérouter
Status evadeOpponent(input_t *input, Command *command, State *state) {
    command->target_left_speed = 0.5;
    command->target_right_speed = -0.5;
    return Status::RUNNING;
}

Status hasBleacherAttached(input_t *input, Command *command, State *state) {
    return state->bleacher_lifted ? Status::SUCCESS : Status::FAILURE;
}

Status isClearOfDroppedBleacher(input_t *input, Command *command, State *state) {
    float x, y, _orientation;
    state->getPositionAndOrientation(x, y, _orientation);
    const auto [bleacher, distance] = state->world.closest_bleacher_in_building_area(x, y);
    return (distance >= 0.4f || std::abs(angle_normalize(_orientation - bleacher.orientation - M_PI)) > to_radians(20))
               ? Status::SUCCESS
               : Status::FAILURE;
}

Status escapeBleacher(input_t *input, Command *command, State *state) {
    command->target_left_speed = -0.3f;
    command->target_right_speed = -0.3f;
    return Status::RUNNING;
}

Status goToClosestBuildingArea(input_t *input, Command *command, State *state) {
    auto seq = sequence(
        //
        [](input_t *input_, Command *command_, State *state_) {
            float x, y, _orientation;
            state_->getPositionAndOrientation(x, y, _orientation);
            const auto [building_area, distance] = state_->world.closest_available_building_area(x, y);

            // Fail if no building areas available
            if (distance == std::numeric_limits<float>::max()) {
                host_printf("No building area\n");
                return Status::FAILURE;
            }

            // Inside a rectangle centered around the building area's orthogonal axis?
            if (auto [local_x, local_y] = building_area.position_in_local_frame(x, y);
                std::abs(local_x) <= BUILDING_AREA_ATTRACTION_HALF_LENGTH &&
                std::abs(local_y) <= BUILDING_AREA_ATTRACTION_HALF_WIDTH) {
                return Status::SUCCESS;
            } else {
                host_printf("Searching\n");
                mcu_printf("BA-SRCH\n");
                state_->world.set_target(TargetType::BuildingAreaWaypoint);
                descend(*command_, *state_);
                command_->shovel = ShovelCommand::SHOVEL_EXTENDED;
                return Status::RUNNING;
            }
        },
        [](input_t *input_, Command *command_, State *state_) {
            float x, y, orientation;
            state_->getPositionAndOrientation(x, y, orientation);
            auto [building_area, distance] = state_->world.closest_available_building_area(x, y);

            constexpr float SLOW_DOWN_DISTANCE = 0.3f;
            constexpr float STOP_DISTANCE = 0.15f;

            if (distance < STOP_DISTANCE) {
                return Status::SUCCESS;
            }

            auto const [wp_x, wp_y] = building_area.waypoint();
            auto const [target_x, target_y] = building_area.available_slot();
            const bool has_arrived =
                stanley_controller(x, y, orientation, wp_x, wp_y, target_x, target_y, target_x - (wp_x - target_x),
                                   target_y - (wp_y - target_y), 0.8f, 1.0f, 0.3f, 200.0f, WHEELBASE_M,
                                   SLOW_DOWN_DISTANCE, &command_->target_left_speed, &command_->target_right_speed);

            if (has_arrived) {
                return Status::SUCCESS;
            } else {
                host_printf("Approaching building area x=%.3f y=%.3f\n", wp_x, wp_y);
                mcu_printf("BA-APP\n");
                command_->shovel = ShovelCommand::SHOVEL_EXTENDED;
                return Status::RUNNING;
            }
        },
        [](input_t *input_, Command *command_, State *state_) {
            float x, y, orientation;
            state_->getPositionAndOrientation(x, y, orientation);
            auto [building_area, distance] = state_->world.closest_available_building_area(x, y);
            auto const [target_x, target_y] = building_area.available_slot();

            const bool has_arrived = pid_controller(x, y, orientation, target_x, target_y, 0.8f, WHEELBASE_M, 0.15,
                                                    &command_->target_left_speed, &command_->target_right_speed);

            if (has_arrived) {
                // Mark the slot as occupied
                building_area.first_available_slot++;
                return Status::SUCCESS;
            } else {
                host_printf("Approaching building area centre x=%.3f y=%.3f\n", target_x, target_y);
                mcu_printf("BA-APPCNT\n");
                command_->shovel = ShovelCommand::SHOVEL_EXTENDED;
                return Status::RUNNING;
            }
        },
        [](input_t *input_, Command *command_, State *state_) {
            host_printf("Building area close enough, dropping the bleacher\n");
            mcu_printf("BA-ARR\n");
            command_->shovel = ShovelCommand::SHOVEL_RETRACTED;
            command_->target_left_speed = 0.f;
            command_->target_right_speed = 0.f;
            state_->bleacher_lifted = false;
            return Status::RUNNING;
        });

    return seq(const_cast<input_t *>(input), command, state);
}

Status gotoClosestBleacher(input_t *input, Command *command, State *state) {
    auto seq = sequence(
        //
        [](input_t *input_, Command *command_, State *state_) {
            float x, y, _orientation;
            state_->getPositionAndOrientation(x, y, _orientation);
            const auto [bleacher, distance] = state_->world.closest_available_bleacher(x, y);

            // Fail if no bleachers available
            if (distance == std::numeric_limits<float>::max()) {
                host_printf("No bleacher\n");
                return Status::FAILURE;
            }

            // Inside a rectangle centered around the bleacher's orthogonal axis?
            if (auto [local_x, local_y] = bleacher.position_in_local_frame(x, y);
                std::abs(local_x) <= BLEACHER_ATTRACTION_HALF_LENGTH &&
                std::abs(local_y) <= BLEACHER_ATTRACTION_HALF_WIDTH) {
                return Status::SUCCESS;
            } else {
                host_printf("Searching\n");
                mcu_printf("BL-SRCH\n");
                state_->world.set_target(TargetType::BleacherWaypoint);
                descend(*command_, *state_);
                return Status::RUNNING;
            }
        },
        [](input_t *input_, Command *command_, State *state_) {
            float x, y, orientation;
            state_->getPositionAndOrientation(x, y, orientation);
            auto [bleacher, distance] = state_->world.closest_available_bleacher(x, y);

            constexpr float SLOW_DOWN_DISTANCE = 0.3f;
            constexpr float STOP_DISTANCE = 0.15f;

            if (distance < STOP_DISTANCE)
                return Status::SUCCESS;

            auto const waypoints = bleacher.waypoints();
            constexpr float MAX = std::numeric_limits<float>::max();
            std::array<float, 3> closest_waypoint = {0, 0, MAX}; // [x, y, squared distance]
            std::array<float, 3> farthest_waypoint = {0, 0, 0};  // [x, y, squared distance]
            for (const auto [wp_x, wp_y] : waypoints) {
                const auto squared_distance = static_cast<float>(std::pow(wp_x - x, 2) + std::pow(wp_y - y, 2));
                if (squared_distance < closest_waypoint[2])
                    closest_waypoint = {wp_x, wp_y, squared_distance};
                if (squared_distance > farthest_waypoint[2])
                    farthest_waypoint = {wp_x, wp_y, squared_distance};
            }

            const bool has_arrived =
                stanley_controller(x, y, orientation, closest_waypoint[0], closest_waypoint[1], bleacher.x, bleacher.y,
                                   farthest_waypoint[0], farthest_waypoint[1], 0.8f, 1.0f, 0.3f, 200.0f, WHEELBASE_M,
                                   SLOW_DOWN_DISTANCE, &command_->target_left_speed, &command_->target_right_speed);

            if (has_arrived) {
                return Status::SUCCESS;
            } else {
                host_printf("Approaching bleacher x=%.3f y=%.3f\n", closest_waypoint[0], closest_waypoint[1]);
                mcu_printf("BL-APP\n");
                return Status::RUNNING;
            }
        },
        [](input_t *input_, Command *command_, State *state_) {
            float x, y, orientation;
            state_->getPositionAndOrientation(x, y, orientation);
            const auto bleacher = state_->world.closest_available_bleacher(x, y).first;

            const bool has_arrived = pid_controller(x, y, orientation, bleacher.x, bleacher.y, 0.8f, WHEELBASE_M, 0.15,
                                                    &command_->target_left_speed, &command_->target_right_speed);

            if (has_arrived) {
                // Remove from the list of available bleachers.
                state_->world.remove_bleacher(bleacher);
                return Status::SUCCESS;
            } else {
                host_printf("Approaching bleacher centre x=%.3f y=%.3f\n", bleacher.x, bleacher.y);
                mcu_printf("BL-APPCNT\n");
                return Status::RUNNING;
            }
        },
        [](input_t *input_, Command *command_, State *state_) {
            host_printf("Bleacher close enough, stopping\n");
            mcu_printf("BL-ARR\n");
            command_->shovel = ShovelCommand::SHOVEL_EXTENDED;
            command_->target_left_speed = 0.f;
            command_->target_right_speed = 0.f;
            state_->bleacher_lifted = true;
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
    state->gameNotStarted();
    return Status::FAILURE;
}

Status isGameActive(input_t *input, Command *command, State *state) {
    return state->elapsedTime(*input) < 90.0f ? Status::SUCCESS : Status::FAILURE;
}

Status isBackstagePhaseNotActive(input_t *input, Command *command, State *state) {
    return state->elapsedTime(*input) > 80.0f ? Status::FAILURE : Status::SUCCESS;
}

Status goToBackstage(input_t *input, Command *command, State *state) {
    host_printf("Aller en backstage\n");
    mcu_printf("BCKSTG\n");
    state->world.set_target(TargetType::BackstageWaypoint);
    descend(*command, *state);

    return Status::RUNNING;
}

Status waitBeforeGame(input_t *input, Command *command, State *state) {
    if (input->blue_button) {
        host_printf("Blue button pressed\n");
        mcu_printf("BUTTON\n");
        state->reset();
    }
    command->target_left_speed = 0.f;
    command->target_right_speed = 0.f;
    command->shovel = ShovelCommand::SHOVEL_RETRACTED;
    return Status::RUNNING;
}

// Attente indéfinie
Status holdAfterEnd(input_t *input, Command *command, State *state) {
    host_printf("Holding after game ends\n");
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
    const bool has_arrived =
        pid_controller(state->imu_x, state->imu_y, state->imu_theta, target_imu_x, target_imu_y, 0.8f, WHEELBASE_M,
                       0.08, &command->target_left_speed, &command->target_right_speed);
    if (has_arrived) {
        state->target_nb++;
        return Status::SUCCESS;
    } else {
        return Status::RUNNING;
    }
}

Status isFlagPhaseCompleted(const input_t *input, Command *command, State *state) {
    if (state->elapsedTime(*input) > 1.0f) {
        return Status::SUCCESS;
    }
    return Status::FAILURE;
}

Status deployFlag(const input_t *input, Command *command, State *state) {
    command->target_left_speed = 0.5f;
    command->target_right_speed = 0.5f;
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
        alternative(isJackRemoved, logAndFail("game-not-started"), waitBeforeGame),
        alternative(isGameActive, logAndFail("game-finished"), holdAfterEnd),
        alternative(isSafe, logAndFail("ensure-safety"), evadeOpponent),
        alternative(isFlagPhaseCompleted, logAndFail("release_flag"), deployFlag),
        alternative(isBackstagePhaseNotActive, logAndFail("go-to-backstage"), goToBackstage),
        alternative(isClearOfDroppedBleacher, logAndFail("escape-bleacher"), escapeBleacher),
        alternative(hasBleacherAttached, logAndFail("pickup-bleacher"), gotoClosestBleacher),
        alternative(logAndFail("drop-bleacher"), goToClosestBuildingArea));
    return root(const_cast<input_t *>(input), command, state);
}
