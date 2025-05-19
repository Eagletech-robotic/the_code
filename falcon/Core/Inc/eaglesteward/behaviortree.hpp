#pragma once

#include "eaglesteward/command.hpp"
#include "eaglesteward/state.hpp"
#include "iot01A/input.h"

#include <functional>
#include <tuple>
#include <type_traits>
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

/* ─────────────  StateNode sans heap, autorisé par GCC  ─────────────── */
template <typename... Fs>
struct StaticStateNode
{
    std::tuple<Fs...> children;                 // lambdas stockées
    std::size_t       cursor    = 0;
    std::uint32_t     last_tick = 0;

    /* Ctor : prend les lambdas à l’init ------------------------------- */
    explicit constexpr StaticStateNode(Fs... fs)
        : children(std::move(fs)...)
    {}

    /* Appelle l’enfant I --------------------------------------------- */
    template <std::size_t I>
    Status call(input_t* in, Command* c, State* s)
    {
        return std::get<I>(children)(in, c, s);
    }

    /* Dispatch sur l’index courant (pliage) -------------------------- */
//    template <std::size_t... Is>
//    Status dispatch(input_t* in, Command* c, State* s,
//                    std::index_sequence<Is...>)
//    {
//        Status r = Status::FAILURE;
//        ((cursor == Is && (r = call<Is>(in, c, s), true)), ...);
//
//        switch (r)
//        {
//            case Status::SUCCESS:
//                ++cursor;
//                if (cursor == sizeof...(Fs)) { cursor = 0; return Status::SUCCESS; }
//                return Status::RUNNING;
//            case Status::RUNNING:
//                return Status::RUNNING;
//            case Status::FAILURE:
//            default:
//                cursor = 0;
//                return Status::FAILURE;
//        }
//    }

    template<std::size_t... Is>
    Status dispatch(input_t* in, Command* c, State* s, std::index_sequence<Is...>)
    {
        Status r = Status::FAILURE;

        [[maybe_unused]]
        bool matched = (
            (cursor == Is &&
             (static_cast<void>(r = call<Is>(in, c, s)), true)) || ...
        );

        switch (r)
        {
            case Status::SUCCESS:
                ++cursor;
                if (cursor == sizeof...(Fs)) { cursor = 0; return Status::SUCCESS; }
                return Status::RUNNING;
            case Status::RUNNING:
                return Status::RUNNING;
            case Status::FAILURE:
            default:
                cursor = 0;
                return Status::FAILURE;
        }
    }


    /* operator() principal ------------------------------------------- */
    Status operator()(input_t* in, Command* c, State* s)
    {
        if (last_tick + 1 < s->bt_tick) cursor = 0;   // reset si raté un tick
        last_tick = s->bt_tick;

        return dispatch(in, c, s,
                        std::make_index_sequence<sizeof...(Fs)>{});
    }
};

/* ─────────────  Usine à nœuds : retourne un lambda façade  ────────── */
template <typename... Fs>
auto statenode(Fs... fs)
{
    static_assert((is_valid_behavior<Fs>() && ...),
                  "lambda bad signature (statenode)");

    /* Instance unique, *statique*, zéro allocation dynamique --------- */
    static StaticStateNode<Fs...> node{ fs... };

    return [](input_t* in, Command* c, State* s) -> Status
    {
        return node(in, c, s);
    };
}


int behaviortree_test();
