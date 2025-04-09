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

// Signature attendue : Status(input_t*, output_t*, state_t*)

template <typename F>
constexpr bool is_valid_behavior() {
    return std::is_invocable_r_v<Status, F, input_t*, output_t*, state_t*>;
}

// Cas de base (une seule lambda)
template<typename F>
auto sequence(F f) {
    static_assert(is_valid_behavior<F>(), "Lambda doesn't match required signature.");
    return [f](input_t* input, output_t* output, state_t* state) -> Status {
        return f(input, output, state);
    };
}

// Cas récursif (plusieurs lambdas)

template<typename F, typename... Rest>
auto sequence(F f, Rest... rest) {
    static_assert(is_valid_behavior<F>(), "Lambda doesn't match required signature.");
    static_assert((is_valid_behavior<Rest>() && ...), "At least one lambda doesn't match required signature.");
    auto next = sequence(rest...);

    return [f, next](input_t* input, output_t* output, state_t* state) -> Status {
        Status result = f(input, output, state);

        switch (result) {
            case Status::SUCCESS:
                return next(input, output, state);
            case Status::FAILURE:
            case Status::RUNNING:
                return result;
        }

        return Status::FAILURE; // Sécurité, devrait jamais arriver
    };
}

// Cas de base : une seule lambda
template<typename F>
auto alternative(F f) {
    static_assert(is_valid_behavior<F>(), "Lambda doesn't match required signature.");
    return [f](input_t* input, output_t* output, state_t* state) -> Status {
        return f(input, output, state);
    };
}

// Cas récursif : plusieurs lambdas
template<typename F, typename... Rest>
auto alternative(F f, Rest... rest) {
    static_assert(is_valid_behavior<F>(), "Lambda doesn't match required signature.");
    static_assert((is_valid_behavior<Rest>() && ...), "At least one lambda doesn't match required signature.");

    auto next = alternative(rest...);

    return [f, next](input_t* input, output_t* output, state_t* state) -> Status {
        Status result = f(input, output, state);

        switch (result) {
            case Status::FAILURE:
                return next(input, output, state); // essayer suivant
            case Status::SUCCESS:
            case Status::RUNNING:
                return result; // succès ou en cours => on s'arrête
        }
        return Status::FAILURE; // sécurité
    };
}

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

    auto sel = alternative(fail, seq,running, success); // on s'arrête sur le 2e

    Status s = sel(&in, &out, &state);
    std::cout << "Final: " << static_cast<int>(s) << "\n";

    return 0;
}

