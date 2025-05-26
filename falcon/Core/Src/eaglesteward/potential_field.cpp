#include "eaglesteward/potential_field.hpp"
#include "utils/angles.hpp"
#include <algorithm>

#include "utils/myprintf.hpp"

bool PotentialField::compute_dijkstra_partial(
    const std::array<std::array<ObstacleType, FIELD_HEIGHT_SQ>, FIELD_WIDTH_SQ> &obstacles,
    BoundedPriorityQueue<PQueueNode, 5'000> &pqueue, const std::function<bool()> &can_continue) {

    if (pqueue.empty())
        return false;

    constexpr int CHECK_INTERVAL = 200;

    constexpr float COST_STRAIGHT = SQUARE_SIZE_M;
    constexpr float COST_DIAG = SQUARE_SIZE_M * 1.414f;

    constexpr float MOVABLE_OBSTACLE_COST_MULTIPLIER = 2.5f;

    struct Step {
        int dx, dy;
        float cost;
    };
    static constexpr Step steps[8] = {{-1, -1, COST_DIAG},    {0, -1, COST_STRAIGHT}, {1, -1, COST_DIAG},
                                      {-1, 0, COST_STRAIGHT}, {1, 0, COST_STRAIGHT},  {-1, 1, COST_DIAG},
                                      {0, 1, COST_STRAIGHT},  {1, 1, COST_DIAG}};

    const ObstacleType *obstaclesField = obstacles.data()->data();
    float *potential = potential_.data()->data();

    for (int it = 0; !pqueue.empty(); ++it) {
        if (it == CHECK_INTERVAL) [[unlikely]] {
            if (can_continue()) {
                it = 0;
            } else {
                return true;
            }
        }

        const PQueueNode &node = pqueue.top();

        const auto x = node.x;
        const auto y = node.y;
        const auto baseDist = node.distance;

        pqueue.pop();

        for (const Step &step : steps) {
            const int newX = x + step.dx;
            const int newY = y + step.dy;

            if (static_cast<uint32_t>(newX) >= FIELD_WIDTH_SQ || static_cast<uint32_t>(newY) >= FIELD_HEIGHT_SQ)
                [[unlikely]]
                continue;

            const int newIdx = newX * FIELD_HEIGHT_SQ + newY;

            if (obstaclesField[newIdx] == ObstacleType::Fixed) [[unlikely]] {
                continue;
            }

            float stepCost = step.cost;

            if (obstaclesField[newIdx] == ObstacleType::Movable) [[unlikely]] {
                stepCost *= MOVABLE_OBSTACLE_COST_MULTIPLIER;
            }

            float cost = stepCost + baseDist;

            float &distSquare = potential[newIdx];
            if (distSquare > cost) {
                distSquare = cost;
                pqueue.emplace(distSquare, newX, newY);
            }
        }
    }

    return false;
}

float PotentialField::finite_potential(int i, int j) const {
    float const v = potential_[i][j];
    if (v != FLT_MAX)
        return v;

    // Look for the neighbour with the lowest finite potential
    float best = FLT_MAX;
    int best_c = 0, best_r = 0;

    for (int c = -1; c <= 1; ++c)
        for (int r = -1; r <= 1; ++r) {
            if (c == 0 && r == 0)
                continue;
            int ii = i + c;
            int jj = j + r;
            if (ii < 0 || ii >= FIELD_WIDTH_SQ || jj < 0 || jj >= FIELD_HEIGHT_SQ)
                continue;
            float neighbour = potential_[ii][jj];
            if (neighbour != FLT_MAX && neighbour < best) {
                best = neighbour;
                best_c = c;
                best_r = r;
            }
        }

    // all neighbours are infinite
    if (best == FLT_MAX)
        return FLT_MAX;

    constexpr float d_cell = SQUARE_SIZE_M;
    return best + d_cell * std::sqrt(float(best_c * best_c + best_r * best_r));
}

float PotentialField::potential_at(float px, float py) const {
    // indices fractionnaires
    float gx = px / SQUARE_SIZE_M;
    float gy = py / SQUARE_SIZE_M;

    int i = static_cast<int>(std::floor(gx));
    int j = static_cast<int>(std::floor(gy));
    float tx = gx - i; // 0..1
    float ty = gy - j; // 0..1

    // bornes
    i = std::clamp(i, 0, FIELD_WIDTH_SQ - 2);
    j = std::clamp(j, 0, FIELD_HEIGHT_SQ - 2);

    // bilinéaire
    float v00 = finite_potential(i, j);
    float v10 = finite_potential(i + 1, j);
    float v01 = finite_potential(i, j + 1);
    float v11 = finite_potential(i + 1, j + 1);

    return (1 - tx) * (1 - ty) * v00 + (tx) * (1 - ty) * v10 + (1 - tx) * (ty)*v01 + (tx) * (ty)*v11;
}

std::pair<float, float> PotentialField::bilinear_gradient(float px, float py) const {
    float gx = px / SQUARE_SIZE_M;
    float gy = py / SQUARE_SIZE_M;

    int i = static_cast<int>(std::floor(gx));
    int j = static_cast<int>(std::floor(gy));

    i = std::clamp(i, 0, FIELD_WIDTH_SQ - 2);
    j = std::clamp(j, 0, FIELD_HEIGHT_SQ - 2);

    float tx = gx - i; // 0..1
    float ty = gy - j;

    float v00 = finite_potential(i, j);
    float v10 = finite_potential(i + 1, j);
    float v01 = finite_potential(i, j + 1);
    float v11 = finite_potential(i + 1, j + 1);

    /* U(tx,ty) = a + b tx + c ty + d tx ty  */
    // float a = v00;
    float b = v10 - v00;
    float c = v01 - v00;
    float d = v11 - v10 - v01 + v00;

    float dU_dtx = b + d * ty;
    float dU_dty = c + d * tx;

    float k = 1.0f / SQUARE_SIZE_M; // chain rule
    return {dU_dtx * k, dU_dty * k};
}

bool PotentialField::gradient_descent(float x, float y, float arrival_distance, float &out_yaw) const {
    constexpr float SMOOTHING_FACTOR = 0.8f; // The closer to 1, the smoother
    constexpr float SLOPE_THRESHOLD = 0.01f;
    static float current_out_yaw = 0.0f;

    out_yaw = current_out_yaw;

    if (potential_at(x, y) <= arrival_distance)
        return true; // Already arrived

    auto [dx, dy] = bilinear_gradient(x, y);

    const float norm = std::hypot(dx, dy);
    if (norm <= SLOPE_THRESHOLD)
        return true;

    const float calculated_yaw = std::atan2(-dy, -dx);

    // Smooth the angle difference
    const float angle_diff = angle_normalize(calculated_yaw - current_out_yaw);
    current_out_yaw = angle_normalize(current_out_yaw + (1 - SMOOTHING_FACTOR) * angle_diff);

    out_yaw = current_out_yaw;
    return false;
}