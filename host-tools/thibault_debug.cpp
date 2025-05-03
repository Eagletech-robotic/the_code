#include "eaglesteward/guidance/thibault.hpp"
#include "eaglesteward/state.hpp"
#include "utils/constants.hpp"
#include "utils/debug.hpp"

extern state_t thibault_state;

int main(int argc, char **argv) {
    config_t config{};

    thibault_top_init(config);

    auto const world = thibault_state.world;
    visualize_potential_field(world.potential(), FIELD_WIDTH_SQ, FIELD_HEIGHT_SQ);

    return 0;
}
