#include "eaglesteward/tof.hpp"

float tof_filter(const state_t &state, float value) {
    if (value < 0.01f || value > 1.9f) {
        return state.filtered_tof_m;
    }
    float rate = 0.04f;
    return (1.0f - rate) * state.filtered_tof_m + (rate * value);
}
