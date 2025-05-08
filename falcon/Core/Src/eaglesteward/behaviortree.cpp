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
