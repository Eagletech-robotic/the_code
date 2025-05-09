#pragma once

#include "eaglesteward/behaviortree.hpp"
#include "eaglesteward/state.hpp"
#include "iot01A/input.h"
#include "eaglesteward/command.hpp"

Status top_behavior(const input_t *input, Command *command, State *state);
