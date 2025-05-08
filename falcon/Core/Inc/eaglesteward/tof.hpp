#pragma once

#include "eaglesteward/state.hpp"

float tof_filter(const State &state, float value);

bool isBleacherPossiblyAtContact(const State &state);

bool isBigThingClose(const State &state);

void updateTofStateMachine(State &state);
