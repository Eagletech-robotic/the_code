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

bool descend(Command &command, State &state, float v_max, float w_max, float r_max, float arrival_distance = 0.01f,
             bool reverseOK = false) {
    constexpr float KP_ROTATION = 50.0f; // Rotation PID's P gain

    auto &world = state.world;
    float target_angle;

    bool has_arrived = world.potential_field_descent(state.robot_x, state.robot_y, arrival_distance, target_angle);
    myprintf("descend %.3f", to_degrees(target_angle));

    if (has_arrived) {
        return true;
    } else {
        // Choose fwd/bwd movement
        auto const angle_diff_fwd = angle_normalize(target_angle - state.robot_theta);
        float angle_diff = angle_diff_fwd; // valeur par défaut (avant)
        float linear_speed = v_max;        // vitesse par défaut (avant)

        if (reverseOK) {
            float angle_diff_rev = angle_normalize(target_angle - (state.robot_theta + M_PI));
            /* Si l’erreur d’orientation est plus petite en marche arrière,
               on choisit le mode arrière et une vitesse linéaire négative.   */
            if (fabsf(angle_diff_rev) < fabsf(angle_diff_fwd)) {
                angle_diff = angle_diff_rev;
                linear_speed = -v_max; // même module, signe négatif
            }
        }

        // Calculate the linear and angular speed
        float angular_speed = KP_ROTATION * angle_diff; // rad/s
        /* Limitation (|w| ≤ w_max, rayon ≥ r_max) ------------------------------ */
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

auto dontMoveUntil = [](float s) {
    return [=](const input_t *input, Command *command, State *state) {
        myprintf("dont move");
        command->target_left_speed = 0.f;
        command->target_right_speed = 0.f;
        return state->elapsedTime(*input) < s ? Status::RUNNING : Status::SUCCESS;
    };
};

auto rotate = [](float angle, float Kp_angle = 250.0f) {
    return [=](const input_t *, Command *command, State *state) {
        myprintf("rotate %.f", to_degrees(angle));
        float error_angle = angle_normalize(angle - state->robot_theta);

        if (fabsf(error_angle) < to_radians(4)) {
            return Status::SUCCESS;
        }

        float w = Kp_angle * error_angle; /* rad/s */
        float halfBase = WHEELBASE_M * 0.5f;
        float v_left = -w * halfBase;
        float v_right = +w * halfBase;

        command->target_left_speed = v_left;
        command->target_right_speed = v_right;
        return Status::RUNNING;
    };
};

Status isSafe(input_t *input, Command *, State *state) {
    if (state->elapsedTime(*input) - state->on_evade_since < 0.5f) {
        return Status::FAILURE;
    }

    if (state->filtered_tof_m < 0.18f) {
        myprintf("FLSAFE\n");
        state->on_evade_since = state->elapsedTime(*input);
        return Status::FAILURE;
    }

    // Use the last interpolated opponent's position, or its last known position if it cannot be interpolated.
    auto const &world = state->world;
    float opponent_x, opponent_y;
    if (!world.opponent_tracker.get_interpolated_position(opponent_x, opponent_y)) {
        opponent_x = world.opponent_x;
        opponent_y = world.opponent_y;
    }

    // Calculate opponent distance and presence on our trajectory.
    float const distance = std::hypot(state->robot_x - opponent_x, state->robot_y - opponent_y);
    bool in_front = isBInFrontOfA(state->robot_x, state->robot_y, state->robot_theta, opponent_x, opponent_y);
    auto const in_trajectory = state->is_moving_forward ? in_front : !in_front;

    myprintf("Safety %.2f [%i %i] %.2f %.2f\n", distance, in_front, state->is_moving_forward, opponent_x, opponent_y);

    // "Not-safe" condition
    if (in_trajectory && distance < 0.45f) {
        myprintf("SFE-DETECT %.2f\n", distance);
        state->on_evade_since = state->elapsedTime(*input);
        return Status::FAILURE;
    }

    state->on_evade_since = -100.0f;
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
            myprintf("SFE-HOLD %2.f\n", state->elapsedTime(*input) - state->on_evade_since);
            command->target_left_speed = 0.f;
            command->target_right_speed = 0.f;

            // Precompute the potential field
            state->world.set_target(TargetType::Evade, state->elapsedTime(*input));

            if (state->elapsedTime(*input) - state->on_evade_since > 3.0f) {
                return Status::SUCCESS;
            }

            return Status::RUNNING;
        };

        auto evade = [this](input_t *input, Command *command, State *state) {
            myprintf("SFE-EVADE\n");
            state->world.set_target(TargetType::Evade, state->elapsedTime(*input));

            if (state->world.potential_at(state->robot_x, state->robot_y) > FLT_MAX / 2.0f) {
                // on ne sait pas ou fuir

                float target_angle;
                state->world.potential_field_descent(state->robot_x, state->robot_y, 0.01, target_angle);

                if (isBInFrontOfA(state->robot_x, state->robot_y, target_angle, state->world.opponent_x,
                                  state->world.opponent_y)) {
                    command->target_left_speed = 0.0f;
                    command->target_right_speed = 0.0f;
                    return Status::RUNNING;
                }
            }

            descend(*command, *state, 0.6f, MAX_ROTATION_SPEED_BLEACHER, 0.0, 0.01, true);
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

struct BackAfterPickup {
    inline static float startTime = 0.0f; // elapsedtime du déclenchement du Safe

    Status operator()(const input_t *input, Command *command, State *state) {

        auto start = [this](input_t *input, Command *command, State *state) {
            startTime = state->elapsedTime(*input);
            command->shovel = ShovelCommand::SHOVEL_EXTENDED;
            state->lock_target(state->robot_x, state->robot_y, state->robot_theta);
            return Status::SUCCESS;
        };

        auto back = [this](input_t *input, Command *command, State *state) {
            command->shovel = ShovelCommand::SHOVEL_EXTENDED;

            int const bleacher_index = state->world.closest_initial_bleacher_index(state->target.x, state->target.y);

            bool straight_backward = false;
            float target_angle = FLT_MAX;

            // Bleachers are numbered from 0 to 9, clockwise from the central right.
            switch (bleacher_index) {
            case 1:
                target_angle = to_radians(45);
                break;
            case 2:
                straight_backward = true;
                break;
            case 3:
                if (state->colour == RobotColour::Yellow)
                    target_angle = to_radians(-45);
                else
                    target_angle = to_radians(90);
                break;
            case 4:
                if (state->colour == RobotColour::Yellow)
                    target_angle = to_radians(135);
                break;
            case 5:
                if (state->colour == RobotColour::Blue)
                    target_angle = to_radians(45);
                break;
            case 6:
                if (state->colour == RobotColour::Blue)
                    target_angle = to_radians(-135);
                else
                    target_angle = to_radians(90);
                break;
            case 7:
                straight_backward = true;
                break;
            case 8:
                target_angle = to_radians(-45);
                break;
            }

            // Straight backward should last this long.
            if (straight_backward && state->elapsedTime(*input) - startTime > 5.0f) {
                return Status::SUCCESS;
            }

            // Skip this bleacher
            if (!straight_backward && target_angle > FLT_MAX / 2.0f) {
                return Status::SUCCESS;
            }

            // Move backward in straight line
            if (straight_backward) {
                myprintf("BA-BCK straight\n");
                command->target_left_speed = -0.5f;
                command->target_right_speed = -0.5f;
                return Status::RUNNING;
            }

            // Otherwise, move backward while rotating to the target angle
            auto const angle_diff = angle_normalize(target_angle - state->robot_theta);

            if (fabs(angle_diff) < to_radians(5)) {
                return Status::SUCCESS;
            }

            myprintf("BA-BCK diff=%.2f\n", to_degrees(angle_diff));
            command->target_left_speed = angle_diff > 0.0f ? -0.5f : 0.0f;
            command->target_right_speed = angle_diff < 0.0f ? -0.5f : 0.0f;
            return Status::RUNNING;
        };

        auto node = statenode(start, back);
        return node(const_cast<input_t *>(input), command, state);
    }
};

Status goToClosestBuildingArea(input_t *input, Command *command, State *state) {
    state->world.set_target(TargetType::BuildingAreaWaypoint, state->elapsedTime(*input));

    constexpr float SPEED_DESCENT = 1.0f;
    constexpr float SPEED_APPROACH = 0.40f;

    auto node = statenode(
        BackAfterPickup{},
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
            descend(*command_, *state_, SPEED_DESCENT, MAX_ROTATION_SPEED_BLEACHER, MAX_ROTATION_RADIUS);
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            command_->shovel = ShovelCommand::SHOVEL_EXTENDED;

            auto const &slot = state_->target;
            auto const [local_x, local_y] = slot.position_in_local_frame(state_->robot_x, state_->robot_y);
            float const distance = std::copysign(std::max(std::fabs(local_x / 2.0f), 0.28f), local_x);
            auto const target_x = slot.x + cos(slot.orientation) * distance;
            auto const target_y = slot.y + sin(slot.orientation) * distance;

            if (pid_controller(state_->robot_x, state_->robot_y, state_->robot_theta, target_x, target_y, SPEED_DESCENT,
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
            const auto building_area = state_->world.closest_building_area(state_->robot_x, state_->robot_y, true);
            if (!building_area) {
                return Status::FAILURE;
            }

            auto const flag_clearance = building_area->is_starting() && building_area->first_available_slot == 0;

            if (pid_controller(state_->robot_x, state_->robot_y, state_->robot_theta, slot.x, slot.y, SPEED_APPROACH,
                               MAX_ROTATION_SPEED_BLEACHER, MAX_ROTATION_RADIUS, WHEELBASE_M,
                               ROBOT_RADIUS - (flag_clearance ? 0.00f : 0.05f), &command_->target_left_speed,
                               &command_->target_right_speed)) {
                building_area->first_available_slot++;
                return Status::SUCCESS;
            }

            myprintf("BA-APPCNT x=%.3f y=%.3f\n", slot.x, slot.y);
            return Status::RUNNING;
        },
        [](input_t *input_, Command *command_, State *state_) {
            command_->shovel = ShovelCommand::SHOVEL_EXTENDED;
            return rotate(angle_normalize(state_->target.orientation + M_PI), 10.0f)(input_, command_, state_);
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
    return state->elapsedTime(*input) > 81.0f ? Status::FAILURE : Status::SUCCESS;
}

Status goToBackstage(input_t *input, Command *command, State *state) {
    command->shovel = ShovelCommand::SHOVEL_RETRACTED;

    auto node = statenode(
        //
        [](input_t *input, Command *command, State *state) {
            myprintf("BCKSTG\n");
            state->world.set_target(TargetType::BackstageWaypoint, state->elapsedTime(*input));

            auto const backstage = state->world.backstage();
            float const dx = state->robot_x - backstage->x;
            float const dy = state->robot_y - backstage->y;

            if (std::abs(dx) < 0.06f && std::abs(dy) < 0.5f) {
                return Status::SUCCESS;
            }

            descend(*command, *state, MAX_SPEED, MAX_ROTATION_SPEED, 0.0f, 0.01f, true);
            return Status::RUNNING;
        },
        rotate(M_PI_2),    //
        dontMoveUntil(96), //
        [](input_t *, Command *command, State *state) {
            float target_x, target_y;
            if (state->colour == RobotColour::Blue) {
                target_x = 3.00f - 0.375f;
                target_y = 2.00f - 0.4f;
            } else {
                target_x = 0.375f;
                target_y = 2.00f - 0.4f;
            }
            const bool has_arrived = pid_controller(state->robot_x, state->robot_y, state->robot_theta, target_x,
                                                    target_y, MAX_SPEED, MAX_ROTATION_SPEED, MAX_ROTATION_RADIUS,
                                                    WHEELBASE_M, // m, entraxe
                                                    0.05f,       // m, distance à l'arrivé pour être arrivé
                                                    &command->target_left_speed, &command->target_right_speed);
            if (has_arrived) {
                return Status::SUCCESS;
            } else {
                return Status::RUNNING;
            }
        },
        holdAfterEnd);

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

    if (descend(*command, *state, MAX_SPEED, MAX_ROTATION_SPEED * 3.0, MAX_ROTATION_RADIUS)) {
        return Status::SUCCESS;
    }

    descend(*command, *state, MAX_SPEED, MAX_ROTATION_SPEED, MAX_ROTATION_RADIUS, 0.10f);
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
    state->bt_tick++; // gestion des statenode

    auto root = sequence( //
        alternative(isJackRemoved, logAndFail("Game-not-started"), waitBeforeGame),
        // alternative(logAndFail("back and forward"), backAndForwardStateNode),
        // alternative(logAndFail("Rectangle descend"), infiniteRectangleDescend),
        alternative(isGameActive, logAndFail("Game-finished"), holdAfterEnd),
        alternative(isSafe, logAndFail("Ensure-safety"), evadeOpponent),
        alternative(isFlagPhaseCompleted, logAndFail("Release-flag"), deployFlag),
        alternative(isBackstagePhaseNotActive, logAndFail("Go-to-backstage"), goToBackstage),
        // alternative(logAndFail("Rectangle statenode"),infiniteRectangleStateNode) ,
        alternative(hasBleacherAttached, logAndFail("Pickup-bleacher"), gotoClosestBleacher),
        alternative(logAndFail("Drop-bleacher"), goToClosestBuildingArea));
    return root(const_cast<input_t *>(input), command, state);
}

void behavior_init(State *state) { controllers_pid_init(&state->pid_theta, &state->pid_speed); }
