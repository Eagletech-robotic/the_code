#include "eaglesteward/guidance/thibault.hpp"
#include "utils/constants.hpp"
#include "utils/debug.hpp"

// Access the potential field defined in thibault.cpp
extern float potential_field[P_FIELD_W][P_FIELD_H];

int main(int argc, char **argv) {
    config_t config;

    thibault_top_init(&config);

    visualize_potential_field(potential_field, P_FIELD_W, P_FIELD_H);

    return 0;
}
