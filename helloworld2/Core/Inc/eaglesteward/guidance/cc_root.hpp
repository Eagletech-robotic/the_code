#pragma once

#include "eaglesteward/behaviortree.hpp"
#include "eaglesteward/state.hpp"
#include "iot01A/input.h"
#include "robotic/command.hpp"

Status cc_root_behavior_tree(input_t *input, Command *command, state_t *state);
