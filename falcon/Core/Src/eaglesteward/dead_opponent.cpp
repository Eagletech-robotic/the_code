#include "eaglesteward/dead_opponent.hpp"
#include "math.h"

float distance(float x1, float y1, float x2, float y2) {
    float const x = x1 - x2;
    float const y = y1 - y2;
    return sqrtf(x * x + y * y);
}

void DeadOpponent::tick(float opponent_x, float opponent_y, float elapsed_time_) {
    if (distance(opponent_x, opponent_y, x, y) > 0.1f) {
        x = opponent_x;
        y = opponent_y;
        last_move_timestamp = elapsed_time_;
    }
    elapsed_time = elapsed_time_;
}

bool DeadOpponent::is_dead() const { return last_move_timestamp + 10.0f < elapsed_time; }
