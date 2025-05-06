#include "eaglesteward/tof.hpp"
#include "eaglesteward/state.hpp"

float tof_filter(State &state, float value) {
    if (value < 0.01f || value > 1.9f) {
        return state.filtered_tof_m;
    }
    float rate = 0.04f;
    float ret = (1.0f - rate) * state.filtered_tof_m + (rate * value);
    return ret;
}
