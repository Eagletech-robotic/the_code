#include "eaglesteward/behavior.hpp"
#include "eaglesteward/behaviortree.hpp"
#include "eaglesteward/constants.hpp"
#include "eaglesteward/state.hpp"
#include "eaglesteward/tof.hpp"
#include "math.h"
#include "robotic/controllers.hpp"
#include "utils/angles.hpp"
#include "utils/myprintf.hpp"

// For use in an alternative node, thus returning Status::FAILURE
auto logAndFail(char const *s) {
    return [s](input_t *, Command *, State *) -> Status {
        myprintf("%s\n", s);
        return Status::FAILURE;
    };
}

bool descend(Command &command, State &state, float max_speed) {
    constexpr float KP_ROTATION = 50.0f;             // Rotation PID's P gain
    constexpr float MAX_ANGULAR_SPEED_LOADED = 0.6f; // Limit when we carry a bleacher. rad/s.
    myprintf("descend");
    auto &world = state.world;
    float min_speed = 0.0f;
    bool is_local_minimum;
    float target_angle;
    world.potential_field_descent(state.robot_x, state.robot_y, is_local_minimum, target_angle);

    if (is_local_minimum) {
        return true;
    } else {
        auto const angle_diff = angle_normalize(target_angle - state.robot_theta);

        // Calculate the linear and angular speed

        auto angular_speed = KP_ROTATION * angle_diff; // rad/s
        if (state.bleacher_lifted) {
            angular_speed = std::clamp(angular_speed, -MAX_ANGULAR_SPEED_LOADED, MAX_ANGULAR_SPEED_LOADED);
            min_speed = 0.01f;
        }
        auto const linear_speed = fabsf(angle_diff) > M_PI_4 ? min_speed : max_speed;
        // Wheel speeds
        constexpr auto HALF_BASE = WHEELBASE_M * 0.5f;
        auto speed_left = linear_speed - angular_speed * HALF_BASE;
        auto speed_right = linear_speed + angular_speed * HALF_BASE;

        // Saturation by the fastest wheel
        auto const abs_fastest_wheel = fmaxf(fabsf(speed_left), fabsf(speed_right));
        if (abs_fastest_wheel > max_speed) {
            float scale = max_speed / abs_fastest_wheel;
            speed_left *= scale;
            speed_right *= scale;
        }

        command.target_left_speed = speed_left;
        command.target_right_speed = speed_right;
        return false;
    }
}

// On n'utilise pas la présence du robot adverse, pour être robuste sur ce sujet
Status isSafe(input_t *, Command *, State *state) {
    if (state->filtered_tof_m < 0.2) {
        // failsafe si tout à merdé avant
        myprintf("Failsafe\n");
        return Status::FAILURE;
    }

//    if (isBigThingClose(*state) &&
//        !isLookingOutwards(3.0f, 2.0f, 0.3f, state->robot_x, state->robot_y, state->robot_theta, 0.01f)) {
//        myprintf("BigThing\n");
//        return Status::FAILURE;
//    }

    return Status::SUCCESS;
}

// Trop proche de l'adversaire, il faut se dérouter
Status evadeOpponent(input_t *, Command *command, State *) {
    command->target_left_speed = 0.5;
    command->target_right_speed = -0.5;
    return Status::RUNNING;
}

Status carryBleacher(input_t *input, Command *command, State *state) {
    if (state->bleacher_lifted) {
        if (state->filtered_tof_m > 0.50f) {
            auto carried_bleacher = state->world.carried_bleacher();

            // The bleacher was dropped: update the state...
            state->bleacher_lifted = false;
            state->world.drop_carried_bleacher();

            // ... and mark the bleacher as uncertain.
            if (carried_bleacher) {
                carried_bleacher->uncertain = true;
                state->world.reset_dijkstra();
            }
        } else {
            // Keep the shovel extended
            command->shovel = ShovelCommand::SHOVEL_EXTENDED;

            // Move the bleacher along with the robot
            if (auto carried_bleacher = state->world.carried_bleacher()) {
                float const shovel_x =
                    state->robot_x + cosf(state->robot_theta) * (SHOVEL_TO_CENTER + BLEACHER_WIDTH / 2);
                float const shovel_y =
                    state->robot_y + sinf(state->robot_theta) * (SHOVEL_TO_CENTER + BLEACHER_WIDTH / 2);
                carried_bleacher->x = shovel_x;
                carried_bleacher->y = shovel_y;
                carried_bleacher->orientation = state->robot_theta;
            }
        }
    }

    return Status::SUCCESS;
}

Status hasBleacherAttached(input_t *, Command *, State *state) {
    return state->bleacher_lifted ? Status::SUCCESS : Status::FAILURE;
}

Status isClearOfDroppedBleacher(input_t *, Command *, State *state) {
    const auto [bleacher, distance] = state->world.closest_bleacher_in_building_area(state->robot_x, state->robot_y);

    if (!bleacher || distance >= 0.4f || state->bleacher_lifted // Do not escape the bleacher we are carrying
    ) {
        return Status::SUCCESS;
    }

    float bearing = std::atan2(bleacher->y - state->robot_y, bleacher->x - state->robot_x);
    bool is_facing_bleacher = std::abs(angle_normalize(state->robot_theta - bearing)) <= to_radians(20);
    return is_facing_bleacher ? Status::FAILURE : Status::SUCCESS;
}

Status escapeBleacher(input_t *, Command *command, State *) {
    command->target_left_speed = -0.3f;
    command->target_right_speed = -0.3f;
    return Status::RUNNING;
}

Status goToClosestBuildingArea(input_t *input, Command *command, State *state) {
    state->world.set_target(TargetType::BuildingAreaWaypoint);
    auto seq = statenode(
        //
        [](input_t *, Command *command_, State *state_) {
            const auto [building_area, distance] =
                state_->world.closest_available_building_area(state_->robot_x, state_->robot_y);

            if (!building_area) {
                return Status::FAILURE;
            }

            auto const waypoint = building_area->waypoint();
            auto const [local_x, local_y] = waypoint.position_in_local_frame(state_->robot_x, state_->robot_y);
            if (fabsf(local_y) < 0.15f) {
                return Status::SUCCESS;
            }

            myprintf("BA-SRCH x=%.3f y=%.3f\n", local_x, local_y);
            descend(*command_, *state_, .8f);
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            const auto [building_area, distance] =
                state_->world.closest_available_building_area(state_->robot_x, state_->robot_y);
            if (!building_area) {
                return Status::FAILURE;
            }

            auto const slot = building_area->available_slot();
            auto const [local_x, local_y] = slot.position_in_local_frame(state_->robot_x, state_->robot_y);
            auto const target_x = slot.x + cos(slot.orientation) * local_x;
            auto const target_y = slot.y + sin(slot.orientation) * local_x;

            if (pid_controller(state_->robot_x, state_->robot_y, state_->robot_theta, target_x, target_y, .8f,
                               WHEELBASE_M, 0.04f, &command_->target_left_speed, &command_->target_right_speed)) {
                return Status::SUCCESS;
            }

            myprintf("BA-APP x=%.3f y=%.3f\n", target_x, target_y);
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            const auto [building_area, distance] =
                state_->world.closest_available_building_area(state_->robot_x, state_->robot_y);
            if (!building_area) {
                return Status::FAILURE;
            }

            auto const slot = building_area->available_slot();
            if (pid_controller(state_->robot_x, state_->robot_y, state_->robot_theta, slot.x, slot.y, .4f, WHEELBASE_M,
                               ROBOT_RADIUS, &command_->target_left_speed, &command_->target_right_speed)) {
                building_area->first_available_slot++;
                return Status::SUCCESS;
            }

            myprintf("BA-APPCNT x=%.3f y=%.3f\n", building_area->x, building_area->y);
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            myprintf("BA-DROP\n");
            command_->shovel = ShovelCommand::SHOVEL_RETRACTED;
            command_->target_left_speed = 0.f;
            command_->target_right_speed = 0.f;
            state_->bleacher_lifted = false;
            state_->world.drop_carried_bleacher();

            return Status::SUCCESS;
        },
        alternative(isClearOfDroppedBleacher, logAndFail("escape-bleacher"), escapeBleacher));

    return seq(const_cast<input_t *>(input), command, state);
}

Status gotoClosestBleacher(input_t *input, Command *command, State *state) {
    state->world.set_target(TargetType::BleacherWaypoint);
    auto seq = statenode(
        [](input_t *, Command *command_, State *state_) {
            const auto [bleacher, distance] =
                state_->world.closest_available_bleacher(state_->robot_x, state_->robot_y);

            if (bleacher) {
                auto [local_x, local_y] = bleacher->position_in_local_frame(state_->robot_x, state_->robot_y);
                if (distance < BLEACHER_WAYPOINT_DISTANCE + 0.10f && fabsf(local_y) < 0.10f) {
                    return Status::SUCCESS;
                }
            };

            myprintf("BL-SRCH\n");
            descend(*command_, *state_, SPEED_MAX);
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            const auto [bleacher, distance] =
                state_->world.closest_available_bleacher(state_->robot_x, state_->robot_y);
            if (!bleacher) {
                return Status::FAILURE;
            }

            auto [local_x, local_y] = bleacher->position_in_local_frame(state_->robot_x, state_->robot_y);
            float const target_x = bleacher->x + cos(bleacher->orientation) * local_x;
            float const target_y = bleacher->y + sin(bleacher->orientation) * local_x;

            if (pid_controller(state_->robot_x, state_->robot_y, state_->robot_theta, target_x, target_y, 1.0f,
                               WHEELBASE_M, 0.04f, &command_->target_left_speed, &command_->target_right_speed)) {
                return Status::SUCCESS;
            }

            myprintf("BL-APP x=%.3f y=%.3f\n", target_x, target_y);
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            const auto [bleacher, distance] =
                state_->world.closest_available_bleacher(state_->robot_x, state_->robot_y);
            if (!bleacher) {
                return Status::FAILURE;
            }

            command_->shovel = ShovelCommand::SHOVEL_EXTENDED;

            if (pid_controller(state_->robot_x, state_->robot_y, state_->robot_theta, bleacher->x, bleacher->y, 0.5f,
                               WHEELBASE_M, 0.15f, &command_->target_left_speed, &command_->target_right_speed)) {
                return Status::SUCCESS;
            }

            myprintf("BL-APPCNT x=%.3f y=%.3f\n", bleacher->x, bleacher->y);
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            auto bleacher = state_->world.closest_available_bleacher(state_->robot_x, state_->robot_y).first;
            state_->bleacher_lifted = true;

            state_->world.carry_bleacher(*bleacher);

            command_->shovel = ShovelCommand::SHOVEL_EXTENDED;
            command_->target_left_speed = 0.f;
            command_->target_right_speed = 0.f;

            myprintf("BL-PKP\n");
            return Status::SUCCESS;
        });

    return seq(const_cast<input_t *>(input), command, state);
}

Status isJackRemoved(input_t *input, Command *, State *state) {
    if (!state->hasGameStarted() && input->jack_removed) {
        state->startGame(input->clock_ms);
    }

    if (input->jack_removed)
        return Status::SUCCESS;

    state->gameNotStarted();
    return Status::FAILURE;
}

Status isGameActive(input_t *input, Command *, State *state) {
    return state->elapsedTime(*input) < 90.0f ? Status::SUCCESS : Status::FAILURE;
}

Status isBackstagePhaseNotActive(input_t *input, Command *, State *state) {
    return state->elapsedTime(*input) > 80.0f ? Status::FAILURE : Status::SUCCESS;
}

Status goToBackstage(input_t *, Command *command, State *state) {
    myprintf("BCKSTG\n");
    state->world.set_target(TargetType::BackstageWaypoint);
    descend(*command, *state, 2.0f);
    command->shovel = ShovelCommand::SHOVEL_RETRACTED;
    return Status::RUNNING;
}

Status waitBeforeGame(input_t *input, Command *command, State *state) {
    if (input->blue_button) {
        myprintf("BUTTON\n");
        state->reset();
    }
    command->target_left_speed = 0.f;
    command->target_right_speed = 0.f;
    command->shovel = ShovelCommand::SHOVEL_RETRACTED;
    return Status::RUNNING;
}

// Attente indéfinie
Status holdAfterEnd(input_t *, Command *command, State *) {
    host_printf("Holding after game ends\n");
    command->target_left_speed = 0.f;
    command->target_right_speed = 0.f;
    command->shovel = ShovelCommand::SHOVEL_RETRACTED;
    return Status::RUNNING;
}

// DEBUG - used by infiniteRectangle
Status gotoTarget(float target_robot_x, float target_robot_y, int target_nb, input_t *, Command *command,
                  State *state) {
    if (state->target_nb != target_nb) {
        return Status::SUCCESS;
    }

    myprintf("B%d\r\n", state->target_nb);
    const bool has_arrived =
        pid_controller(state->robot_x, state->robot_y, state->robot_theta, target_robot_x, target_robot_y,
                       2.0f,        // m/s Vmax 3.0 est le max
                       WHEELBASE_M, // m, entraxe
                       0.08,        // m, distance à l'arrivée pour être arrivé
                       &command->target_left_speed, &command->target_right_speed);
    if (has_arrived) {
        state->target_nb++;
        return Status::SUCCESS;
    } else {
        return Status::RUNNING;
    }
}

Status isFlagPhaseCompleted(const input_t *input, Command *, State *state) {
    if (state->elapsedTime(*input) > 1.5f) {
        return Status::SUCCESS;
    }
    return Status::FAILURE;
}

Status deployFlag(const input_t *, Command *command, State *) {
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

Status gotoTarget2(float target_robot_x, float target_robot_y, int target_nb, input_t *, Command *command,
                   State *state) {
    const bool has_arrived =
        pid_controller(state->robot_x, state->robot_y, state->robot_theta, target_robot_x, target_robot_y,
                       2.0f,        // m/s Vmax 3.0 est le max
                       WHEELBASE_M, // m, entraxe
                       0.08,        // m, distance à l'arrivé pour être arrivé
                       &command->target_left_speed, &command->target_right_speed);
    if (has_arrived) {
        return Status::SUCCESS;
    } else {
        return Status::RUNNING;
    }
}

Status infiniteRectangleStateNode(const input_t *input, Command *command, State *state) {
    auto node = statenode([](input_t *input_, Command *command_,
                             State *state_) { return gotoTarget2(0.75f, 0.30, 0, input_, command_, state_); },
                          [](input_t *input_, Command *command_, State *state_) {
                              return gotoTarget2(1.50, 0.30, 1, input_, command_, state_);
                          },
                          [](input_t *input_, Command *command_, State *state_) {
                              return gotoTarget2(1.5, 1.2, 2, input_, command_, state_);
                          },
                          [](input_t *input_, Command *command_, State *state_) {
                              return gotoTarget2(0.75, 1.2, 3, input_, command_, state_);
                          });

    return node(const_cast<input_t *>(input), command, state);
}

Status gotoDescend(const char *name, Command *command, State *state, TargetType target) {
    state->world.set_target(target);
    myprintf("%s\n", name);
    if (descend(*command, *state, 2.0)) {
        return Status::SUCCESS;
    }
    return Status::RUNNING;
}

Status infiniteRectangleDescend(const input_t *input, Command *command, State *state) {

    auto node = statenode([](input_t *input_, Command *command_,
                             State *state_) { return gotoDescend("0", command_, state_, TargetType::TestPoint0); },
                          [](input_t *input_, Command *command_, State *state_) {
                              return gotoDescend("1", command_, state_, TargetType::TestPoint1);
                          },
                          [](input_t *input_, Command *command_, State *state_) {
                              return gotoDescend("2", command_, state_, TargetType::TestPoint2);
                          },
                          [](input_t *input_, Command *command_, State *state_) {
                              return gotoDescend("3", command_, state_, TargetType::TestPoint3);
                          });

    return node(const_cast<input_t *>(input), command, state);
}

// Top level node
Status top_behavior(const input_t *input, Command *command, State *state) {
    updateTofStateMachine(*state);
    state->bt_tick++;
    auto root = sequence( //
        alternative(isJackRemoved, logAndFail("Game-not-started"), waitBeforeGame),
        // alternative(logAndFail("Rectangle statenode"),infiniteRectangleStateNode) ,
        // alternative(logAndFail("Rectangle descend"), infiniteRectangleDescend),
        alternative(isGameActive, logAndFail("Game-finished"), holdAfterEnd),
        carryBleacher, // Keep this action before evasive maneuvers
        alternative(isSafe, logAndFail("Ensure-safety"), evadeOpponent),
        alternative(isFlagPhaseCompleted, logAndFail("Release-flag"), deployFlag),
        alternative(isBackstagePhaseNotActive, logAndFail("Go-to-backstage"), goToBackstage),
        alternative(hasBleacherAttached, logAndFail("Pickup-bleacher"), gotoClosestBleacher),
        alternative(logAndFail("Drop-bleacher"), goToClosestBuildingArea));
    return root(const_cast<input_t *>(input), command, state);
}

void behavior_init(State *state) { controllers_pid_init(&state->pid_theta, &state->pid_speed); }
