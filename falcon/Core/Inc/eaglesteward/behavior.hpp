#pragma once

#include "eaglesteward/behaviortree.hpp"
#include "eaglesteward/command.hpp"
#include "eaglesteward/state.hpp"
#include "iot01A/input.h"

Status top_behavior(const input_t *input, Command *command, State *state);
