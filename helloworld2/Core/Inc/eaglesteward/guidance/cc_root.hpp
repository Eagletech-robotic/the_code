#pragma once

#include "eaglesteward/behaviortree.hpp"
#include "eaglesteward/state.hpp"
#include "iot01A/input.h"
#include "robotic/command.hpp"

Status cc_infinite_rectangle(const input_t *input, Command *command, state_t *func_state);
Status cc_root_behavior_tree(const input_t *input, Command *command, state_t *state);
