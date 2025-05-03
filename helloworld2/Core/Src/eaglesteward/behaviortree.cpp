/*
 * behaviortree.cpp
 *
 *  Created on: Apr 8, 2025
 *      Author: nboulay
 */

#include <tuple>
#include <type_traits>
// #include <iostream> // code gros, ralenti le download sur cible
#include "eaglesteward/behaviortree.hpp"
#include <stdio.h>

Status behavior_as_function(input_t *, Command *, state_t *) {
    printf("function\n");
    return Status::SUCCESS;
}

int behaviortree_test() {
    auto ok = [](input_t *, Command *, state_t *) {
        printf("OK\n");
        return Status::SUCCESS;
    };

    auto wait = [](input_t *, Command *, state_t *) {
        puts("WAIT\n");
        return Status::RUNNING;
    };

    // auto invalide = [](int x) { return Status::SUCCESS; }; // ❌ Provoque une erreur de compilation si inclue
    auto seq = sequence(ok, behavior_as_function, wait); // ✅

    input_t input;
    Command command{};
    state_t state;

    seq(&input, &command, &state);

    auto fail = [](input_t *, Command *, state_t *) {
        puts("FAIL\n");
        return Status::FAILURE;
    };

    auto running = [](input_t *, Command *, state_t *) {
        puts("RUNNING\n");
        return Status::RUNNING;
    };

    auto success = [](input_t *, Command *, state_t *) {
        puts("SUCCESS\n");
        return Status::SUCCESS;
    };

    auto sel = alternative(fail, seq, running, success, [](input_t *, Command *, state_t *) {
        puts("SUCCESS\n");
        return Status::SUCCESS;
    }); // on s'arrête sur le 2e

    Status s = sel(&input, &command, &state);
    // std::cout << "Final: " << static_cast<int>(s) << "\n";
    printf("Final : %d\n", static_cast<int>(s));
    return 0;
}
