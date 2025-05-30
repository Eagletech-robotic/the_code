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
    constexpr float INFINITE_PENALTY = 1.0f * SQUARE_SIZE_M; // Discourage robot from staying in infinite area
    return best + d_cell * std::sqrt(float(best_c * best_c + best_r * best_r)) + INFINITE_PENALTY;
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

std::pair<float, float> PotentialField::interpolated_gradient(float px, float py) const {
    // Convert to grid coordinates, in a grid system shifted lower left by 0.5 SQUARE_SIZE_M
    float gx = px / SQUARE_SIZE_M + 0.5f;
    float gy = py / SQUARE_SIZE_M + 0.5f;
    int i = static_cast<int>(std::floor(gx));
    int j = static_cast<int>(std::floor(gy));
    float frac_x = gx - i;
    float frac_y = gy - j;

    // Clamp to valid range
    i = std::clamp(i, 0, FIELD_WIDTH_SQ - 2);
    j = std::clamp(j, 0, FIELD_HEIGHT_SQ - 2);

    // Compute gradients at the 4 cell centers
    float cx00 = (i - 0.5f) * SQUARE_SIZE_M;
    float cy00 = (j - 0.5f) * SQUARE_SIZE_M;
    auto [gx00, gy00] = bilinear_gradient(cx00, cy00);

    float cx10 = (i + 0.5f) * SQUARE_SIZE_M;
    float cy10 = (j - 0.5f) * SQUARE_SIZE_M;
    auto [gx10, gy10] = bilinear_gradient(cx10, cy10);

    float cx01 = (i - 0.5f) * SQUARE_SIZE_M;
    float cy01 = (j + 0.5f) * SQUARE_SIZE_M;
    auto [gx01, gy01] = bilinear_gradient(cx01, cy01);

    float cx11 = (i + 0.5f) * SQUARE_SIZE_M;
    float cy11 = (j + 0.5f) * SQUARE_SIZE_M;
    auto [gx11, gy11] = bilinear_gradient(cx11, cy11);

    // Bilinear interpolation of gradients
    float w00 = (1 - frac_x) * (1 - frac_y);
    float w10 = frac_x * (1 - frac_y);
    float w01 = (1 - frac_x) * frac_y;
    float w11 = frac_x * frac_y;

    float dx = w00 * gx00 + w10 * gx10 + w01 * gx01 + w11 * gx11;
    float dy = w00 * gy00 + w10 * gy10 + w01 * gy01 + w11 * gy11;

    return {dx, dy};
}

bool PotentialField::gradient_descent(float x, float y, float arrival_distance, float &out_yaw) const {
    constexpr float SMOOTHING_FACTOR = 0.8f; // The closer to 1, the smoother
    constexpr float EPSILON = 1e-4f;

    static float current_out_yaw = 0.0f;
    out_yaw = current_out_yaw;

    float current_potential = potential_at(x, y);

    if (current_potential <= arrival_distance)
        return true; // Already arrived

    float calculated_yaw;
    if (current_potential >= FLT_MAX * 0.5f) { // Interpolated potential may not be strictly equal to FLT_MAX
        // We are at an infinite potential point
        auto [target_x, target_y] = find_nearest_finite_potential(x, y);

        if (std::fabs(target_x - x) < EPSILON && std::fabs(target_y - y) < EPSILON)
            return true; // No finite potential found anywhere

        calculated_yaw = std::atan2(target_y - y, target_x - x);
    } else {
        auto [dx, dy] = interpolated_gradient(x, y);

        if (std::fabs(dx) < EPSILON && std::fabs(dy) < EPSILON)
            return true;

        calculated_yaw = std::atan2(-dy, -dx);
    }

    // Smooth the angle difference
    const float angle_diff = angle_normalize(calculated_yaw - current_out_yaw);
    current_out_yaw = angle_normalize(current_out_yaw + (1 - SMOOTHING_FACTOR) * angle_diff);

    out_yaw = current_out_yaw;
    return false;
}

std::pair<float, float> PotentialField::find_nearest_finite_potential(float x, float y) const {
    // Convert to grid coordinates
    int center_i = static_cast<int>(x / SQUARE_SIZE_M);
    int center_j = static_cast<int>(y / SQUARE_SIZE_M);

    // Clamp to grid bounds
    center_i = std::clamp(center_i, 0, FIELD_WIDTH_SQ - 1);
    center_j = std::clamp(center_j, 0, FIELD_HEIGHT_SQ - 1);

    // If current cell is finite, return current position
    if (potential_[center_i][center_j] != FLT_MAX) {
        return {x, y};
    }

    // Search in expanding squares
    float best_potential = FLT_MAX;
    float best_x = x;
    float best_y = y;
    float best_dist_sq = FLT_MAX;

    // Maximum search radius (in grid cells)
    const int max_radius = std::max(FIELD_WIDTH_SQ, FIELD_HEIGHT_SQ);

    for (int radius = 1; radius < max_radius; ++radius) {
        bool found_finite = false;

        // Check all cells at this radius
        for (int di = -radius; di <= radius; ++di) {
            for (int dj = -radius; dj <= radius; ++dj) {
                // Only check cells on the perimeter of the square
                if (std::abs(di) != radius && std::abs(dj) != radius)
                    continue;

                int i = center_i + di;
                int j = center_j + dj;

                // Check bounds
                if (i < 0 || i >= FIELD_WIDTH_SQ || j < 0 || j >= FIELD_HEIGHT_SQ)
                    continue;

                float pot = potential_[i][j];
                if (pot != FLT_MAX) {
                    found_finite = true;

                    // Calculate actual position of this cell center
                    float cell_x = (i + 0.5f) * SQUARE_SIZE_M;
                    float cell_y = (j + 0.5f) * SQUARE_SIZE_M;

                    // Calculate squared distance
                    float dx = cell_x - x;
                    float dy = cell_y - y;
                    float dist_sq = dx * dx + dy * dy;

                    // Update best if this is closer or same distance with lower potential
                    if (dist_sq < best_dist_sq || (std::abs(dist_sq - best_dist_sq) < 1e-6f && pot < best_potential)) {
                        best_potential = pot;
                        best_x = cell_x;
                        best_y = cell_y;
                        best_dist_sq = dist_sq;
                    }
                }
            }
        }

        // If we found any finite potential at this radius, we're done
        if (found_finite) {
            return {best_x, best_y};
        }
    }

    // No finite potential found - return original position
    return {x, y};
}