#pragma once

#include "eaglesteward/command.hpp"
#include "eaglesteward/state.hpp"
#include "iot01A/input.h"

#include <functional>

// Type de retour du comportement

enum class Status { SUCCESS, FAILURE, RUNNING };

// Signature attendue : Status(input_t*, Command*, State*)

template <typename F> constexpr bool is_valid_behavior() {
    return std::is_invocable_r_v<Status, F, input_t *, Command *, State *>;
}

// Cas de base (une seule lambda)
template <typename F> auto sequence(F f) {
    static_assert(is_valid_behavior<F>(), "Lambda doesn't match required signature.");
    return [f](input_t *input, Command *command, State *state) -> Status { return f(input, command, state); };
}

// Cas récursif (plusieurs lambdas)

template <typename F, typename... Rest> auto sequence(F f, Rest... rest) {
    static_assert(is_valid_behavior<F>(), "Lambda doesn't match required signature.");
    static_assert((is_valid_behavior<Rest>() && ...), "At least one lambda doesn't match required signature.");
    auto next = sequence(rest...);

    return [f, next](input_t *input, Command *command, State *state) -> Status {
        Status result = f(input, command, state);

        switch (result) {
        case Status::SUCCESS:
            return next(input, command, state);
        case Status::FAILURE:
        case Status::RUNNING:
            return result;
        }

        return Status::FAILURE; // Sécurité, devrait jamais arriver
    };
}

// Cas de base : une seule lambda
template <typename F> auto alternative(F f) {
    static_assert(is_valid_behavior<F>(), "Lambda doesn't match required signature.");
    return [f](input_t *input, Command *command, State *state) -> Status { return f(input, command, state); };
}

// Cas récursif : plusieurs lambdas
template <typename F, typename... Rest> auto alternative(F f, Rest... rest) {
    static_assert(is_valid_behavior<F>(), "Lambda doesn't match required signature.");
    static_assert((is_valid_behavior<Rest>() && ...), "At least one lambda doesn't match required signature.");

    auto next = alternative(rest...);

    return [f, next](input_t *input, Command *command, State *state) -> Status {
        Status result = f(input, command, state);

        switch (result) {
        case Status::FAILURE:
            return next(input, command, state); // essayer suivant
        case Status::SUCCESS:
        case Status::RUNNING:
            return result; // succès ou en cours => on s'arrête
        }
        return Status::FAILURE; // sécurité
    };
}

int behaviortree_test();
