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

float descend(Command &command, State &state, float max_speed, float *potential) {
    constexpr float KP_ROTATION = 50.0f;             // Rotation PID's P gain
    constexpr float MAX_ANGULAR_SPEED_LOADED = 2.0f; // Limit when we carry a bleacher. rad/s.

    auto &world = state.world;

    bool is_local_minimum;
    float target_angle;
    *potential = world.potential_field_descent(state.robot_x, state.robot_y, is_local_minimum, target_angle);
    myprintf("descend %.3f", *potential);
    if (is_local_minimum) {
        return true;
    } else {
        auto const angle_diff = angle_normalize(target_angle - state.robot_theta);

        // Calculate the linear and angular speed
        auto const linear_speed = fabsf(angle_diff) > M_PI_4 ? 0.0f : max_speed;
        auto angular_speed = KP_ROTATION * angle_diff; // rad/s
        if (state.bleacher_lifted)
            angular_speed = std::clamp(angular_speed, -MAX_ANGULAR_SPEED_LOADED, MAX_ANGULAR_SPEED_LOADED);

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

float opponentDistance(State *state) {
    float x = state->robot_x - state->world.opponent_x;
    float y = state->robot_y - state->world.opponent_y;
    return sqrtf(x * x + y * y);
}

// On n'utilise pas la présence du robot adverse, pour être robuste sur ce sujet
Status isSafe(input_t *, Command *, State *state) {
    if (state->filtered_tof_m < 0.2) {
        // failsafe si tout à merdé avant
        myprintf("Failsafe\n");
        return Status::FAILURE;
    }
    float d = opponentDistance(state);
    if (d < 0.40) {
        myprintf("opp at %.2f\n", d);
        return Status::FAILURE;
    }

    //    if (isBigThingClose(*state) &&
    //        !isLookingOutwards(3.0f, 2.0f, 0.3f, state->robot_x, state->robot_y, state->robot_theta, 0.01f)) {
    //        myprintf("BigThing\n");
    //        return Status::FAILURE;
    //    }

    return Status::SUCCESS;
}

// --- Gestion isSafe

struct Safe {
    inline static float startTime = 0.0f; // elapsedtime du déclenchement du Safe

    Status operator()(const input_t *input, Command *command, State *state) {

        // 1️⃣ lambdas locales, lisibles, sans piège
        auto detection = [this](input_t *input, Command *command, State *state) {
            startTime = state->elapsedTime(*input);
            myprintf("detection");
            command->target_left_speed = 0.f;
            command->target_right_speed = 0.f;
            return Status::SUCCESS;
        };

        auto threeSecond = [this](input_t *input, Command *command, State *state) {
            myprintf("three second");
            command->target_left_speed = 0.f;
            command->target_right_speed = 0.f;

            if (state->elapsedTime(*input) - startTime > 3.0) {
                return Status::SUCCESS;
            }
            return Status::RUNNING;
        };

        auto evasion = [this](input_t *input, Command *command, State *state) {
            myprintf("evasion");
            float p;
            descend(*command, *state, .6f, &p);
            return Status::RUNNING;
        };

        auto node = statenode(detection, threeSecond, evasion);

        return node(const_cast<input_t *>(input), command, state);
    }
};

// Trop proche de l'adversaire, il faut se dérouter
Status evadeOpponent(input_t *input, Command *command, State *state) { return Safe{}(input, command, state); }

Status carryBleacher(input_t *input, Command *command, State *state) {
    if (state->bleacher_lifted) {
        if (state->filtered_tof_m > 0.50f) {
            auto carried_bleacher = state->world.carried_bleacher();

            // The bleacher was dropped: update the state...
            state->bleacher_lifted = false;
            state->target = nullptr;
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
                host_printf("No building area\n");
                return Status::FAILURE;
            }
            float p;
            if (descend(*command_, *state_, .8f, &p)) {
                // vitesse plus lente pour ne rien lacher
                return Status::SUCCESS;
            }
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            // Aller en face
            auto [building_area, distance] =
                state_->world.closest_available_building_area(state_->robot_x, state_->robot_y);
            auto const [wp_x, wp_y] = building_area->waypoint();
            myprintf("waypoint");
            if (pid_controller(state_->robot_x, state_->robot_y, state_->robot_theta, wp_x, wp_y, .8f, WHEELBASE_M,
                               0.04f, &command_->target_left_speed, &command_->target_right_speed)) {
                return Status::SUCCESS;
            }
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            // Aller en face
            auto [building_area, distance] =
                state_->world.closest_available_building_area(state_->robot_x, state_->robot_y);
            auto const [target_x, target_y] = building_area->available_slot();
            myprintf("slot");
            if (pid_controller(state_->robot_x, state_->robot_y, state_->robot_theta, target_x, target_y, .8f,
                               WHEELBASE_M, 0.04f, &command_->target_left_speed, &command_->target_right_speed)) {
                building_area->first_available_slot++;
                return Status::SUCCESS;
            }
            host_printf("Approaching building area x=%.3f y=%.3f\n", target_x, target_y);

            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            myprintf("BA-DROP\n");
            command_->shovel = ShovelCommand::SHOVEL_RETRACTED;
            command_->target_left_speed = 0.f;
            command_->target_right_speed = 0.f;
            state_->bleacher_lifted = false;
            state_->target = nullptr;
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
            // descente de gradient vers le plus proche
            const auto [bleacher, distance] =
                state_->world.closest_available_bleacher(state_->robot_x, state_->robot_y);

            if (!bleacher) {
                host_printf("No bleacher\n");
                return Status::FAILURE;
            }

            state_->target = bleacher;
            myprintf("BL-SRCH\n");
            float p;
            if (descend(*command_, *state_, 1.0f, &p)) {
                host_printf("Minimum\n");
                return Status::SUCCESS;
            } else {
                return Status::RUNNING;
            }
        },
        [](input_t *, Command *command_, State *state_) {
            // aller en face du gradin
            const auto [bleacher, distance] =
                state_->world.closest_available_bleacher(state_->robot_x, state_->robot_y);
            // const auto bleacher = state_->target;

            auto [local_x, local_y] = bleacher->position_in_local_frame(state_->robot_x, state_->robot_y);

            float const target_x = bleacher->x + cos(bleacher->orientation) * local_x;
            float const target_y = bleacher->y + sin(bleacher->orientation) * local_x;
            if (pid_controller(state_->robot_x, state_->robot_y, state_->robot_theta, target_x, target_y, 0.8f,
                               WHEELBASE_M, 0.04f, &command_->target_left_speed, &command_->target_right_speed)) {
                return Status::SUCCESS;
            }
            return Status::RUNNING;
        },
        [](input_t *, Command *command_, State *state_) {
            // aller vers le gradin
            // const auto bleacher = state_->target;
            const auto [bleacher, distance] =
                state_->world.closest_available_bleacher(state_->robot_x, state_->robot_y);

            const bool has_arrived =
                pid_controller(state_->robot_x, state_->robot_y, state_->robot_theta, bleacher->x, bleacher->y, 0.8f,
                               WHEELBASE_M, 0.15, &command_->target_left_speed, &command_->target_right_speed);
            command_->shovel = ShovelCommand::SHOVEL_EXTENDED;
            if (has_arrived) {
                host_printf("has_arrived\n");
                return Status::SUCCESS;
            } else {
                host_printf("BL-APPCNT x=%.3f y=%.3f\n", bleacher->x, bleacher->y);
                mcu_printf("BL-APPCNT\n");
                return Status::RUNNING;
            }
        },
        [](input_t *, Command *command_, State *state_) {
            // On va ailleurs !
            myprintf("BL-PKP\n");

            auto bleacher = state_->world.closest_available_bleacher(state_->robot_x, state_->robot_y).first;
            state_->bleacher_lifted = true;

            state_->world.carry_bleacher(*bleacher);

            command_->shovel = ShovelCommand::SHOVEL_EXTENDED;
            command_->target_left_speed = 0.f;
            command_->target_right_speed = 0.f;

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
    return state->elapsedTime(*input) > 80.0f ? Status::FAILURE : Status::SUCCESS;
}

Status goToBackstageDescend(input_t *, Command *command, State *state) {
    myprintf("BCKSTG\n");
    state->world.set_target(TargetType::BackstageWaypoint);
    float potential;
    bool ret = descend(*command, *state, 2.0f, &potential);
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
    return [=](const input_t *input, Command *command, State *state) {
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
        target_x = 3.0 - .45;
        target_y = 2.0 - .3;
    } else {
        target_x = .45;
        target_y = 2.0 - .3;
    }
    const bool has_arrived = pid_controller(state->robot_x, state->robot_y, state->robot_theta, target_x, target_y,
                                            1.0f,        // m/s Vmax 3.0 est le max
                                            WHEELBASE_M, // m, entraxe
                                            0.1,         // m, distance à l'arrivé pour être arrivé
                                            &command->target_left_speed, &command->target_right_speed);
    if (has_arrived) {
        return Status::SUCCESS;
    } else {
        return Status::RUNNING;
    }
}

Status goToBackstage(input_t *input, Command *command, State *state) {
    command->shovel = ShovelCommand::SHOVEL_RETRACTED;
    auto node = statenode(goToBackstageDescend, rotate(90.0f), dontMoveUntil(87), gotoBackstageLine, holdAfterEnd);

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
    float p;
    if (descend(*command, *state, 2.0, &p)) {
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
        // alternative(logAndFail("Rectangle descend"), infiniteRectangleDescend),
        alternative(isGameActive, logAndFail("Game-finished"), holdAfterEnd),
        carryBleacher, // Keep this action before evasive maneuvers
        alternative(isSafe, logAndFail("Ensure-safety"), evadeOpponent),
        alternative(isFlagPhaseCompleted, logAndFail("Release-flag"), deployFlag),
        alternative(isBackstagePhaseNotActive, logAndFail("Go-to-backstage"), goToBackstage),
        // alternative(logAndFail("Rectangle statenode"),infiniteRectangleStateNode) ,
        alternative(hasBleacherAttached, logAndFail("Pickup-bleacher"), gotoClosestBleacher),
        alternative(logAndFail("Drop-bleacher"), goToClosestBuildingArea));
    return root(const_cast<input_t *>(input), command, state);
}

void behavior_init(State *state) { controllers_pid_init(&state->pid_theta, &state->pid_speed); }
