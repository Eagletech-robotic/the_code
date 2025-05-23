#pragma once
#include <array>
#include <cstdint>

struct RollingHistory {
    static constexpr int SIZE = 125;
    std::array<float, SIZE> dx{}, dy{}, dtheta{};
    int idx{0};

    void clear() {
        dx.fill(0.f);
        dy.fill(0.f);
        dtheta.fill(0.f);
        idx = 0;
    }

    void push(float delta_x, float delta_y, float delta_theta) {
        dx[idx] = delta_x;
        dy[idx] = delta_y;
        dtheta[idx] = delta_theta;
        idx = (idx + 1) % SIZE;
    }

    void integrateLastSteps(int n, float &out_dx, float &out_dy, float &out_dtheta) const {
        out_dx = out_dy = out_dtheta = 0.f;
        for (int k = 1; k <= n && k <= SIZE; ++k) {
            int j = (idx - k + SIZE) % SIZE;
            out_dx += dx[j];
            out_dy += dy[j];
            out_dtheta += dtheta[j];
        }
    }
};
