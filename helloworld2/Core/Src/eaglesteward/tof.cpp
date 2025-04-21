#include "eaglesteward/tof.hpp"

float tof_filter(const state_t &state, float value) {
    if (value == 0.0f || value == 2.0f) {
        return state.filtered_tof_m;
    }

    return 0.7f * state.filtered_tof_m + (0.3f * value);
}
