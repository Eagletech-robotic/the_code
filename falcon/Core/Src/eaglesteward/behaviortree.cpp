#include "eaglesteward/behaviortree.hpp"
#include "utils/myprintf.hpp"
#include <stdio.h>
#include <tuple>
#include <type_traits>

Status behavior_as_function(input_t *, Command *, State *) {
    printf("function\n");
    return Status::SUCCESS;
}

int behaviortree_test() {
    auto ok = [](input_t *, Command *, State *) {
        printf("OK\n");
        return Status::SUCCESS;
    };

    auto wait = [](input_t *, Command *, State *) {
        puts("WAIT\n");
        return Status::RUNNING;
    };

    // auto invalide = [](int x) { return Status::SUCCESS; }; // ❌ Provoque une erreur de compilation si inclue
    auto seq = sequence(ok, behavior_as_function, wait); // ✅

    input_t input;
    Command command{};
    State state;

    seq(&input, &command, &state);

    auto fail = [](input_t *, Command *, State *) {
        puts("FAIL\n");
        return Status::FAILURE;
    };

    auto running = [](input_t *, Command *, State *) {
        puts("RUNNING\n");
        return Status::RUNNING;
    };

    auto success = [](input_t *, Command *, State *) {
        puts("SUCCESS\n");
        return Status::SUCCESS;
    };

    auto sel = alternative(fail, seq, running, success, [](input_t *, Command *, State *) {
        puts("SUCCESS\n");
        return Status::SUCCESS;
    }); // on s'arrête sur le 2e

    Status s = sel(&input, &command, &state);
    // std::cout << "Final: " << static_cast<int>(s) << "\n";
    printf("Final : %d\n", static_cast<int>(s));
    return 0;
}


int statenode_test()
{
    /* Étape 1 : succès immédiat -------------------------------------------- */
    auto step1 = [](input_t*, Command*, State*) {
        puts("STEP-1  (SUCCESS)");
        return Status::SUCCESS;
    };

    /* Étape 2 : reste en RUNNING 2 ticks avant SUCCESS ---------------------- */
    auto step2 = [](input_t*, Command*, State*) {
        static int cnt = 0;
        if (cnt < 2) {
            printf("STEP-2  (RUNNING %d)\n", cnt + 1);
            ++cnt;
            return Status::RUNNING;
        }
        puts("STEP-2  (SUCCESS)");
        cnt = 0;
        return Status::SUCCESS;
    };

    /* Étape 3 : succès final ------------------------------------------------ */
    auto step3 = [](input_t*, Command*, State*) {
        puts("STEP-3  (SUCCESS)");
        return Status::SUCCESS;
    };

    /* StateNode sans heap (= séquence mémorisée) --------------------------- */
    auto state_seq = statenode(step1, step2, step3);

    /* ---------------------------------------------------------------------- */
    input_t input{};
    Command  cmd{};
    State    st{};

    puts("\n=== StateNode demo ===");

    for (int tick = 0; tick < 6; ++tick)
    {
        st.bt_tick++;                 // nouveau tick global

        /* Simule une priorité supérieure au tick 3 : on NE PASSE PAS par state_seq */
        if (tick == 3) {
            puts(">>> autre branche prioritaire, StateNode ignoré ce tick");
            continue;
        }

        Status s = state_seq(&input, &cmd, &st);

        printf("tick %d → Status %d\n", tick, (int)s);
    }
    return 0;
}
