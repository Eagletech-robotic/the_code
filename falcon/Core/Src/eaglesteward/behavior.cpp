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

bool descend(Command &command, State &state, float v_max, float w_max, float r_max, float arrival_distance = 0.01f) {
    constexpr float KP_ROTATION = 50.0f; // Rotation PID's P gain

    auto &world = state.world;
    float target_angle;

    bool has_arrived = world.potential_field_descent(state.robot_x, state.robot_y, arrival_distance, target_angle);
    myprintf("descend %.3f", to_degrees(target_angle));

    if (has_arrived) {
        return true;
    } else {
        auto const angle_diff = angle_normalize(target_angle - state.robot_theta);

        // Calculate the linear and angular speed
        float angular_speed = KP_ROTATION * angle_diff; // rad/s
        float linear_speed = v_max;
        /* Limitation de la vitesse angulaire ------------------------------------*/
        limit_vw(&linear_speed, &angular_speed, w_max, r_max);

        // Wheel speeds
        constexpr auto HALF_BASE = WHEELBASE_M * 0.5f;
        auto speed_left = linear_speed - angular_speed * HALF_BASE;
        auto speed_right = linear_speed + angular_speed * HALF_BASE;

        // Saturation by the fastest wheel
        auto const abs_fastest_wheel = fmaxf(fabsf(speed_left), fabsf(speed_right));
        if (abs_fastest_wheel > v_max) {
            float scale = v_max / abs_fastest_wheel;
            speed_left *= scale;
            speed_right *= scale;
        }

        command.target_left_speed = speed_left;
        command.target_right_speed = speed_right;
        return false;
    }
}

Status isSafe(input_t *, Command *, State *state) {
    if (state->filtered_tof_m < 0.18f) {
        myprintf("FLSAFE\n");
        return Status::FAILURE;
    }

    // If the opponent position is known, check the distance
    if (state->world.opponent_x != 0.0f && state->world.opponent_y != 0.0f) {
        float const x = state->robot_x - state->world.opponent_x;
        float const y = state->robot_y - state->world.opponent_y;
        float const opponent_distance = sqrtf(x * x + y * y);

        if (opponent_distance < 0.40f) {
            myprintf("SFE-DETECT %.2f\n", opponent_distance);
            return Status::FAILURE;
        }
    }
    return Status::SUCCESS;
}

// --- Gestion isSafe
struct Safe {
    inline static float startTime = 0.0f; // elapsedtime du déclenchement du Safe

    Status operator()(const input_t *input, Command *command, State *state) {
        if (state->bleacher_lifted)
            command->shovel = ShovelCommand::SHOVEL_EXTENDED;

        auto saveTime = [this](input_t *input, Command *command, State *state) {
            startTime = state->elapsedTime(*input);
            command->target_left_speed = 0.f;
            command->target_right_speed = 0.f;
            return Status::SUCCESS;
        };

        auto hold = [this](input_t *input, Command *command, State *state) {
            myprintf("SFE-HOLD\n");
            command->target_left_speed = 0.f;
            command->target_right_speed = 0.f;

            if (state->elapsedTime(*input) - startTime > 3.0) {
                return Status::SUCCESS;
            }
            return Status::RUNNING;
        };

        auto evade = [this](input_t *, Command *command, State *state) {
            myprintf("SFE-EVADE\n");
            descend(*command, *state, 0.6f, MAX_ROTATION_SPEED, MAX_ROTATION_RADIUS);
            return Status::RUNNING;
        };

        auto node = statenode(saveTime, hold, evade);

        return node(const_cast<input_t *>(input), command, state);
    }
};

// Trop proche de l'adversaire, il faut se dérouter
Status evadeOpponent(input_t *input, Command *command, State *state) { return Safe{}(input, command, state); }

Status hasBleacherAttached(input_t *, Command *, State *state) {
    return state->bleacher_lifted ? Status::SUCCESS : Status::FAILURE;
}

Status gotoClosestBleacher(input_t *input, Command *command, State *state) {
    state->world.set_target(TargetType::BleacherWaypoint, state->elapsedTime(*input));

    auto node = statenode(
        [](input_t *, Command *command_, State *state_) {
            const auto [bleacher, distance] =
                state_->world.closest_available_bleacher(state_->robot_x, state_->robot_y);

            if (bleacher) {
                auto [local_x, local_y] = bleacher->position_in_local_frame(state_->robot_x, state_->robot_y);
                if (fabsf(local_x) < BLEACHER_WAYPOINT_DISTANCE + 0.10f && fabsf(local_y) < 0.10f) {
                    state_->lock_target(bleacher->x, bleacher->y, bleacher->orientation);
                    return Status::SUCCESS;
                }
            }

            myprintf("BL-SRCH\n");
            descend(*command_, *state_, MAX_SPEED, MAX_ROTATION_SPEED, MAX_ROTATION_RADIUS);
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            auto const &bleacher = state_->target;
            auto [local_x, local_y] = bleacher.position_in_local_frame(state_->robot_x, state_->robot_y);
            float const distance = std::copysign(std::max(std::fabs(local_x / 2.0f), 0.28f), local_x);
            float const target_x = bleacher.x + cos(bleacher.orientation) * distance;
            float const target_y = bleacher.y + sin(bleacher.orientation) * distance;

            if (pid_controller(state_->robot_x, state_->robot_y, state_->robot_theta, target_x, target_y, MAX_SPEED,
                               MAX_ROTATION_SPEED, MAX_ROTATION_RADIUS, WHEELBASE_M, 0.12f,
                               &command_->target_left_speed, &command_->target_right_speed)) {
                return Status::SUCCESS;
            }

            myprintf("BL-APP x=%.3f y=%.3f\n", target_x, target_y);
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            auto const &bleacher = state_->target;

            if (pid_controller(state_->robot_x, state_->robot_y, state_->robot_theta, bleacher.x, bleacher.y, MAX_SPEED,
                               MAX_ROTATION_SPEED, MAX_ROTATION_RADIUS, WHEELBASE_M, 0.25f,
                               &command_->target_left_speed, &command_->target_right_speed)) {
                return Status::SUCCESS;
            }

            myprintf("BL-APPCNT1 x=%.3f y=%.3f\n", bleacher.x, bleacher.y);
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            auto const &bleacher = state_->target;
            command_->shovel = ShovelCommand::SHOVEL_EXTENDED;

            if (state_->filtered_tof_m > 0.50f) {
                // No bleacher is here: remove from the world and move on.
                printf("BL-NONE\n");
                state_->world.remove_bleacher(state_->target.x, state_->target.y);
                state_->release_target();
                return Status::FAILURE;
            }

            if (pid_controller(state_->robot_x, state_->robot_y, state_->robot_theta, bleacher.x, bleacher.y, 0.25f,
                               MAX_ROTATION_SPEED, MAX_ROTATION_RADIUS, WHEELBASE_M, 0.16f,
                               &command_->target_left_speed, &command_->target_right_speed)) {
                state_->world.remove_bleacher(state_->target.x, state_->target.y);
                state_->release_target();
                state_->bleacher_lifted = true;
                return Status::SUCCESS;
            }

            myprintf("BL-APPCNT2 x=%.3f y=%.3f\n", bleacher.x, bleacher.y);
            return Status::RUNNING;
        });

    return node(const_cast<input_t *>(input), command, state);
}

Status goToClosestBuildingArea(input_t *input, Command *command, State *state) {
    state->world.set_target(TargetType::BuildingAreaWaypoint, state->elapsedTime(*input));

    auto node = statenode(
        //
        [](input_t *, Command *command_, State *state_) {
            const auto building_area = state_->world.closest_building_area(state_->robot_x, state_->robot_y, true);
            if (!building_area) {
                return Status::FAILURE;
            }
            command_->shovel = ShovelCommand::SHOVEL_EXTENDED;

            auto const waypoint = building_area->waypoint();
            auto const [local_x, local_y] = waypoint.position_in_local_frame(state_->robot_x, state_->robot_y);
            if (fabsf(local_x) < 0.10f && fabsf(local_y) < 0.15f) {
                auto const slot = building_area->available_slot();
                state_->lock_target(slot.x, slot.y, slot.orientation);
                return Status::SUCCESS;
            }

            myprintf("BA-SRCH x=%.3f y=%.3f\n", waypoint.x, waypoint.y);
            descend(*command_, *state_, 0.8f, MAX_ROTATION_SPEED_BLEACHER, MAX_ROTATION_RADIUS);
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            command_->shovel = ShovelCommand::SHOVEL_EXTENDED;

            auto const &slot = state_->target;
            auto const [local_x, local_y] = slot.position_in_local_frame(state_->robot_x, state_->robot_y);
            float const distance = std::copysign(std::max(std::fabs(local_x / 2.0f), 0.28f), local_x);
            auto const target_x = slot.x + cos(slot.orientation) * distance;
            auto const target_y = slot.y + sin(slot.orientation) * distance;

            if (pid_controller(state_->robot_x, state_->robot_y, state_->robot_theta, target_x, target_y, 0.8f,
                               MAX_ROTATION_SPEED_BLEACHER, MAX_ROTATION_RADIUS, WHEELBASE_M, 0.12f,
                               &command_->target_left_speed, &command_->target_right_speed)) {
                return Status::SUCCESS;
            }

            myprintf("BA-APP x=%.3f y=%.3f\n", target_x, target_y);
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            command_->shovel = ShovelCommand::SHOVEL_EXTENDED;

            auto const &slot = state_->target;
            if (pid_controller(state_->robot_x, state_->robot_y, state_->robot_theta, slot.x, slot.y, 0.25f,
                               MAX_ROTATION_SPEED_BLEACHER, MAX_ROTATION_RADIUS, WHEELBASE_M, ROBOT_RADIUS - 0.05f,
                               &command_->target_left_speed, &command_->target_right_speed)) {
                const auto building_area = state_->world.closest_building_area(state_->robot_x, state_->robot_y, true);
                if (building_area)
                    building_area->first_available_slot++;

                return Status::SUCCESS;
            }

            myprintf("BA-APPCNT x=%.3f y=%.3f\n", slot.x, slot.y);
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            auto const &slot = state_->target;
            auto const [local_x, local_y] = slot.position_in_local_frame(state_->robot_x, state_->robot_y);

            if (fabsf(local_x) >= ROBOT_RADIUS + 0.15) {
                state_->bleacher_lifted = false;
                return Status::SUCCESS;
            }

            myprintf("BA-DROP %.3f %.3f\n", local_x, local_y);
            command_->target_left_speed = -0.5f;
            command_->target_right_speed = -0.5f;
            return Status::RUNNING;
        });

    return node(const_cast<input_t *>(input), command, state);
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

Status isBackstagePhaseNotActive(input_t *input, Command *, State *state) {
    // 85s PAMIs start
    // 100s End of game
    return state->elapsedTime(*input) > 87.0f ? Status::FAILURE : Status::SUCCESS;
}

Status goToBackstageDescend(input_t *input, Command *command, State *state) {
    myprintf("BCKSTG\n");
    state->world.set_target(TargetType::BackstageWaypoint, state->elapsedTime(*input));

    if (descend(*command, *state, MAX_SPEED, MAX_ROTATION_SPEED, MAX_ROTATION_RADIUS, 0.10f)) {
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
                                            MAX_SPEED, MAX_ROTATION_SPEED, MAX_ROTATION_RADIUS,
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
                       0.6f, // m/s Vmax 3.0 est le max
                       MAX_ROTATION_SPEED, MAX_ROTATION_RADIUS,
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

Status gotoDescend(const char *name, const input_t *input, Command *command, State *state, TargetType target) {
    state->world.set_target(target, state->elapsedTime(*input));
    myprintf("%s\n", name);

    if (descend(*command, *state, MAX_SPEED, MAX_ROTATION_SPEED, MAX_ROTATION_RADIUS)) {
        return Status::SUCCESS;
    }
    return Status::RUNNING;
}

Status infiniteRectangleDescend(const input_t *input, Command *command, State *state) {
    auto node = statenode( //
        [](input_t *input, Command *command_, State *state_) {
            return gotoDescend("0", input, command_, state_, TargetType::TestPoint0);
        },
        [](input_t *input, Command *command_, State *state_) {
            return gotoDescend("1", input, command_, state_, TargetType::TestPoint1);
        },
        [](input_t *input, Command *command_, State *state_) {
            return gotoDescend("2", input, command_, state_, TargetType::TestPoint2);
        },
        [](input_t *input, Command *command_, State *state_) {
            return gotoDescend("3", input, command_, state_, TargetType::TestPoint3);
        });

    return node(const_cast<input_t *>(input), command, state);
}

// Top level node
Status top_behavior(const input_t *input, Command *command, State *state) {
    updateTofStateMachine(*state);
    state->bt_tick++;
    auto root = sequence( //
        alternative(isJackRemoved, logAndFail("Game-not-started"), waitBeforeGame),
        // alternative(logAndFail("back and forward"), backAndForwardStateNode),
        // alternative(logAndFail("Rectangle statenode"),infiniteRectangleStateNode) ,
        // alternative(logAndFail("Rectangle descend"), infiniteRectangleDescend),
        alternative(isGameActive, logAndFail("Game-finished"), holdAfterEnd),
        alternative(isSafe, logAndFail("Ensure-safety"), evadeOpponent),
        alternative(isFlagPhaseCompleted, logAndFail("Release-flag"), deployFlag),
        alternative(isBackstagePhaseNotActive, logAndFail("Go-to-backstage"), goToBackstage),
        alternative(hasBleacherAttached, logAndFail("Pickup-bleacher"), gotoClosestBleacher),
        alternative(logAndFail("Drop-bleacher"), goToClosestBuildingArea));
    return root(const_cast<input_t *>(input), command, state);
}

void behavior_init(State *state) { controllers_pid_init(&state->pid_theta, &state->pid_speed); }
