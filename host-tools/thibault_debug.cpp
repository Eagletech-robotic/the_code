#include "eaglesteward/state.hpp"
#include "utils/constants.hpp"
#include "utils/debug.hpp"

int main(int argc, char **argv) {
    State state;
    visualize_potential_field(state.world.potential_ready(), FIELD_WIDTH_SQ, FIELD_HEIGHT_SQ);

    return 0;
}
