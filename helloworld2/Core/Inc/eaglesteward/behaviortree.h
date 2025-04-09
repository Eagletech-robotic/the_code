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
#include <functional>



// Type de retour du comportement

enum class Status {
    SUCCESS,
    FAILURE,
    RUNNING
};

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
int behaviortree_test();
