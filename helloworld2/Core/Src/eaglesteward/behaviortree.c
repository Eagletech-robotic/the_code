/*
 * behaviortree.cpp
 *
 *  Created on: Apr 8, 2025
 *      Author: nboulay
 */

#include <tuple>
#include <type_traits>
#include <iostream>
#include "eaglesteward/behaviortree.h"



Status behavior_as_function(input_t*, output_t*, state_t*) {
	std::cout << "function\n";
	return Status::SUCCESS;
}

int behaviortree_test() {
    auto ok = [](input_t*, output_t*, state_t*) {
        std::cout << "OK\n";
        return Status::SUCCESS;
    };

    auto wait = [](input_t*, output_t*, state_t*) {
        std::cout << "WAIT\n";
        return Status::RUNNING;
    };

     //auto invalide = [](int x) { return Status::SUCCESS; }; // ❌ Provoque une erreur de compilation si inclue
     auto seq = sequence(ok, behavior_as_function, wait); // ✅

    input_t in;
    output_t out;
    state_t state;

    seq(&in, &out, &state);

     auto fail = [](input_t*, output_t*, state_t*) {
        std::cout << "FAIL\n";
        return Status::FAILURE;
    };

    auto running = [](input_t*, output_t*, state_t*) {
        std::cout << "RUNNING\n";
        return Status::RUNNING;
    };

    auto success = [](input_t*, output_t*, state_t*) {
        std::cout << "SUCCESS\n";
        return Status::SUCCESS;
    };

    auto sel = alternative(fail, seq,running, success, [](input_t*, output_t*, state_t*) {
        std::cout << "SUCCESS\n";
        return Status::SUCCESS;
    }); // on s'arrête sur le 2e

    Status s = sel(&in, &out, &state);
    std::cout << "Final: " << static_cast<int>(s) << "\n";

    return 0;
}

