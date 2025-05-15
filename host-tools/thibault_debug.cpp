#include "eaglesteward/state.hpp"
#include "robotic/constants.hpp"
#include "utils/debug.hpp"

int main(int argc, char **argv) {
    State state;
    state.world.set_target(TargetType::BuildingAreaWaypoint);
    state.world.do_all_calculations_LONG();
    visualize_potential_field(state.world.potential_ready(), FIELD_WIDTH_SQ, FIELD_HEIGHT_SQ);

    return 0;
}
