/*
 * behaviortree.h
 *
 *  Created on: Apr 8, 2025
 *      Author: nboulay
 */
#pragma once

#include "iot01A/input.h"
#include "iot01A/output.h"
#include "eaglesteward/state.h"

// Type de retour du comportement

enum class Status {
    SUCCESS,
    FAILURE,
    RUNNING
};

// ex :  auto root = alternative(fail, seq,running, success);
//       Status s = sel(&in, &out, &state);
template<typename F> auto sequence(F f);
template<typename F, typename... Rest> auto sequence(F f, Rest... rest) ;
template<typename F> auto alternative(F f);
template<typename F, typename... Rest> auto alternative(F f, Rest... rest);

int behaviortree_test();
