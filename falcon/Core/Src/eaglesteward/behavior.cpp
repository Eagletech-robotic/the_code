#include "eaglesteward/behavior.hpp"
#include "eaglesteward/behaviortree.hpp"
#include "eaglesteward/constants.hpp"
#include "eaglesteward/state.hpp"
#include "eaglesteward/tof.hpp"
#include "robotic/controllers.hpp"
#include "utils/angles.hpp"
#include "utils/myprintf.hpp"
#include <cmath>

// For use in an alternative node, thus returning Status::FAILURE
auto logAndFail(char const *s) {
    return [s](input_t *, Command *, State *) -> Status {
        myprintf("%s\n", s);
        return Status::FAILURE;
    };
}

bool descend(Command &command, State &state, float max_speed, float *potential) {
    constexpr float KP_ROTATION = 50.0f;             // Rotation PID's P gain
    constexpr float MAX_ANGULAR_SPEED_LOADED = 2.0f; // Limit when we carry a bleacher. rad/s.

    auto &world = state.world;
    float min_speed = 0.0f;
    bool is_local_minimum;
    float target_angle;

    *potential = world.potential_field_descent(state.robot_x, state.robot_y, is_local_minimum, target_angle);
    myprintf("descend %.3f %.3f", *potential, to_degrees(target_angle));

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

Status checkLostBleacher(input_t *, Command *, State *state) {
    if (state->bleacher_lifted && state->filtered_tof_m > 0.50f) {
        // The bleacher was dropped: update the state...
        state->bleacher_lifted = false;
    }
    return Status::SUCCESS;
}

Status hasBleacherAttached(input_t *, Command *, State *state) {
    return state->bleacher_lifted ? Status::SUCCESS : Status::FAILURE;
}

Status goToClosestBuildingArea(input_t *input, Command *command, State *state) {
    state->world.set_target(TargetType::BuildingAreaWaypoint);

    auto check_lost_bleacher = [](input_t *, Command *command_, State *state_) {
        if (state_->bleacher_lifted && state_->filtered_tof_m > 0.50f) {
            // The bleacher was dropped: update the state...
            state_->bleacher_lifted = false;
            command_->shovel = ShovelCommand::SHOVEL_RETRACTED;
            return Status::FAILURE;
        }
        command_->shovel = ShovelCommand::SHOVEL_EXTENDED;
        return Status::SUCCESS;
    };

    auto go_to_building_area = statenode(
        //
        [](input_t *, Command *command_, State *state_) {
            const auto building_area = state_->world.closest_building_area(state_->robot_x, state_->robot_y, true);
            if (!building_area) {
                return Status::FAILURE;
            }

            auto const waypoint = building_area->waypoint();
            auto const [local_x, local_y] = waypoint.position_in_local_frame(state_->robot_x, state_->robot_y);
            if (fabsf(local_y) < 0.15f) {
                return Status::SUCCESS;
            }

            myprintf("BA-SRCH x=%.3f y=%.3f\n", waypoint.x, waypoint.y);
            float potential;
            descend(*command_, *state_, .8f, &potential);
            command_->shovel = ShovelCommand::SHOVEL_EXTENDED;
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            const auto building_area = state_->world.closest_building_area(state_->robot_x, state_->robot_y, true);
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
            command_->shovel = ShovelCommand::SHOVEL_EXTENDED;
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            const auto building_area = state_->world.closest_building_area(state_->robot_x, state_->robot_y, true);
            if (!building_area) {
                return Status::FAILURE;
            }

            auto const slot = building_area->available_slot();
            if (pid_controller(state_->robot_x, state_->robot_y, state_->robot_theta, slot.x, slot.y, .4f, WHEELBASE_M,
                               ROBOT_RADIUS, &command_->target_left_speed, &command_->target_right_speed)) {
                myprintf("goToClosestBuildingArea - bleacher_lifted = false\n");
                state_->bleacher_lifted = false;
                building_area->first_available_slot++;
                return Status::SUCCESS;
            }

            myprintf("BA-APPCNT x=%.3f y=%.3f\n", building_area->x, building_area->y);
            command_->shovel = ShovelCommand::SHOVEL_EXTENDED;
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            const auto building_area = state_->world.closest_building_area(state_->robot_x, state_->robot_y, false);
            if (!building_area) {
                return Status::SUCCESS;
            }

            auto const slot = building_area->available_slot();
            auto const [local_x, local_y] = slot.position_in_local_frame(state_->robot_x, state_->robot_y);
            if (fabsf(local_y) <= 0.15f) {
                return Status::SUCCESS;
            }

            myprintf("BA-DROP %.3f %.3f\n", local_x, local_y);
            command_->shovel = ShovelCommand::SHOVEL_RETRACTED;
            command_->target_left_speed = -0.3f;
            command_->target_right_speed = -0.3f;
            return Status::RUNNING;
        });

    return sequence(check_lost_bleacher, go_to_building_area)(const_cast<input_t *>(input), command, state);
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
            float potential;
            descend(*command_, *state_, SPEED_MAX, &potential);
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
                state_->bleacher_lifted = true;
                state_->world.bleachers_.remove(*bleacher);
                return Status::SUCCESS;
            }

            myprintf("BL-APPCNT x=%.3f y=%.3f\n", bleacher->x, bleacher->y);
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *) {
            myprintf("gotoClosestBleacher - bleacher_lifted = true\n");
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
    return state->elapsedTime(*input) < 100.0f ? Status::SUCCESS : Status::FAILURE;
}

// Attente indéfinie
Status holdAfterEnd(input_t *, Command *command, State *) {
    host_printf("Holding after game ends\n");
    command->target_left_speed = 0.f;
    command->target_right_speed = 0.f;
    command->shovel = ShovelCommand::SHOVEL_RETRACTED;
    return Status::RUNNING;
}

// --- Backstage 10 pts
//  85 s PAMI démarre
//  90 s le robot ne peux plus bouger
// 100 s les PAMI stop

Status isBackstagePhaseNotActive(input_t *input, Command *, State *state) {
    return state->elapsedTime(*input) > 87.0f ? Status::FAILURE : Status::SUCCESS;
}

Status goToBackstageDescend(input_t *, Command *command, State *state) {
    myprintf("BCKSTG\n");
    state->world.set_target(TargetType::BackstageWaypoint);
    float potential;
    bool ret = descend(*command, *state, SPEED_MAX, &potential);
    if (ret || (potential < 0.13)) {
        return Status::SUCCESS;
    }
    return Status::RUNNING;
}

auto dontMoveUntil = [](float s) {
    return [=](const input_t *input, Command *command, State *state) {
        myprintf("dont move");
        command->target_left_speed = 0.f;
        command->target_right_speed = 0.f;
        return state->elapsedTime(*input) < s ? Status::RUNNING : Status::SUCCESS;
    };
};

auto rotate = [](float a) {
    return [=](const input_t *, Command *command, State *state) {
        myprintf("rotate %.f", a);
        float desired_angle = a * DEG_TO_RAD;
        float error_angle = angle_normalize(desired_angle - state->robot_theta);

        if (fabsf(error_angle) < 1 * DEG_TO_RAD) {
            return Status::SUCCESS;
        }

        const float Kp_angle = 250.0f;    /* angle    → vitesse angulaire  */
        float w = Kp_angle * error_angle; /* rad/s */
        float halfBase = WHEELBASE_M * 0.5f;
        float v_left = -w * halfBase;
        float v_right = +w * halfBase;

        command->target_left_speed = v_left;
        command->target_right_speed = v_right;
        return Status::RUNNING;
    };
};

Status gotoBackstageLine(input_t *, Command *command, State *state) {
    float target_x, target_y;
    if (state->colour == RobotColour::Blue) {
        target_x = 3.00f - 0.375f;
        target_y = 2.00f - 0.15f;
    } else {
        target_x = 0.375f;
        target_y = 2.00f - 0.15f;
    }
    const bool has_arrived = pid_controller(state->robot_x, state->robot_y, state->robot_theta, target_x, target_y,
                                            1.0f,        // m/s Vmax 3.0 est le max
                                            WHEELBASE_M, // m, entraxe
                                            0.05f,       // m, distance à l'arrivé pour être arrivé
                                            &command->target_left_speed, &command->target_right_speed);
    if (has_arrived) {
        return Status::SUCCESS;
    } else {
        return Status::RUNNING;
    }
}

Status goToBackstage(input_t *input, Command *command, State *state) {
    command->shovel = ShovelCommand::SHOVEL_RETRACTED;
    auto node = statenode(goToBackstageDescend, rotate(90.0f), dontMoveUntil(96), gotoBackstageLine, holdAfterEnd);

    return node(const_cast<input_t *>(input), command, state);
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

Status gotoTarget2(float target_robot_x, float target_robot_y, int /*target_nb*/, input_t *, Command *command,
                   State *state) {
    const bool has_arrived =
        pid_controller(state->robot_x, state->robot_y, state->robot_theta, target_robot_x, target_robot_y,
                       0.6f,        // m/s Vmax 3.0 est le max
                       WHEELBASE_M, // m, entraxe
                       0.08,        // m, distance à l'arrivé pour être arrivé
                       &command->target_left_speed, &command->target_right_speed);
    if (has_arrived) {
        return Status::SUCCESS;
    } else {
        return Status::RUNNING;
    }
}

Status backAndForwardStateNode(const input_t *input, Command *command, State *state) {
    auto node = statenode([](input_t *input_, Command *command_,
                             State *state_) { return gotoTarget2(0.6f, 0.60, 0, input_, command_, state_); },
                          [](input_t *input_, Command *command_, State *state_) {
                              return gotoTarget2(3.0f - .60f, 0.60, 1, input_, command_, state_);
                          });

    return node(const_cast<input_t *>(input), command, state);
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
    float potential;
    if (descend(*command, *state, 2.0, &potential)) {
        return Status::SUCCESS;
    }
    return Status::RUNNING;
}

Status infiniteRectangleDescend(const input_t *input, Command *command, State *state) {
    auto node = statenode([](input_t *, Command *command_,
                             State *state_) { return gotoDescend("0", command_, state_, TargetType::TestPoint0); },
                          [](input_t *, Command *command_, State *state_) {
                              return gotoDescend("1", command_, state_, TargetType::TestPoint1);
                          },
                          [](input_t *, Command *command_, State *state_) {
                              return gotoDescend("2", command_, state_, TargetType::TestPoint2);
                          },
                          [](input_t *, Command *command_, State *state_) {
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
        alternative(isSafe, logAndFail("Ensure-safety"), evadeOpponent),
        alternative(isFlagPhaseCompleted, logAndFail("Release-flag"), deployFlag),
        // alternative(logAndFail("back and forward"), backAndForwardStateNode),
        alternative(isBackstagePhaseNotActive, logAndFail("Go-to-backstage"), goToBackstage),
        alternative(hasBleacherAttached, logAndFail("Pickup-bleacher"), gotoClosestBleacher),
        alternative(logAndFail("Drop-bleacher"), goToClosestBuildingArea));
    return root(const_cast<input_t *>(input), command, state);
}

void behavior_init(State *state) { controllers_pid_init(&state->pid_theta, &state->pid_speed); }
