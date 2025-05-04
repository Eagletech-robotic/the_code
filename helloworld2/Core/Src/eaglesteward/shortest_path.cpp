#include "eaglesteward/shortest_path.hpp"

#include <utils/sized_array.hpp>

Coord compute_distances(const std::array<std::array<GridSquare, FIELD_HEIGHT_SQ>, FIELD_WIDTH_SQ> &grid, Coord root,
                        std::array<std::array<uint16_t, FIELD_HEIGHT_SQ>, FIELD_WIDTH_SQ> &outDistances) {
    std::array<SizedArray<Coord, FIELD_WIDTH_SQ * FIELD_HEIGHT_SQ>, 2> terminalNodes{};
    terminalNodes[0].push_back(root);

    int parity = 0;
    int currDst = 1;
    while (!terminalNodes[parity].empty()) {
        for (const Coord &node : terminalNodes[parity]) {
            for (int dx = -1; dx <= 1; dx++) {
                for (int dy = -1; dy <= 1; dy++) {
                    if (dx == 0 && dy == 0)
                        continue;

                    const int newX = node.x + dx;
                    const int newY = node.y + dy;

                    if (newX < 0 || newX >= FIELD_WIDTH_SQ || newY < 0 || newY >= FIELD_HEIGHT_SQ)
                        continue;

                    if (grid[newX][newY] == GridSquare::Obstacle)
                        continue;

                    if (grid[newX][newY] == GridSquare::Target) {
                        outDistances[newX][newY] = currDst;
                        return {static_cast<uint8_t>(newX), static_cast<uint8_t>(newY)};
                    }

                    if (outDistances[newX][newY] == UINT16_MAX) {
                        outDistances[newX][newY] = currDst;
                        terminalNodes[1 - parity].push_back({static_cast<uint8_t>(newX), static_cast<uint8_t>(newY)});
                    }
                }
            }
        }

        terminalNodes[parity].clear();
        parity = 1 - parity;
        currDst++;
    }

   // throw std::runtime_error("Target not found");
}

void compute_shortest_path(const std::array<std::array<GridSquare, FIELD_HEIGHT_SQ>, FIELD_WIDTH_SQ> &grid,
                           const Coord root, SizedArray<Coord, FIELD_WIDTH_SQ * FIELD_HEIGHT_SQ> &outPath) {
    // Dynamic programming to compute the shortest path
    std::array<std::array<uint16_t, FIELD_HEIGHT_SQ>, FIELD_WIDTH_SQ> distances = {};
    for (int i = 0; i < FIELD_WIDTH_SQ; ++i) {
        distances[i].fill(UINT16_MAX);
    }
    distances[root.x][root.y] = 0;

    const Coord target = compute_distances(grid, root, distances);

    outPath.clear();

    Coord curr = target;
    while (distances[curr.x][curr.y] != 0) {
        bool found = false;
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0)
                    continue;

                const int newX = curr.x + dx;
                const int newY = curr.y + dy;

                if (newX < 0 || newX >= FIELD_WIDTH_SQ || newY < 0 || newY >= FIELD_HEIGHT_SQ)
                    continue;

                if (distances[newX][newY] == distances[curr.x][curr.y] - 1) {
                    curr = {static_cast<uint8_t>(newX), static_cast<uint8_t>(newY)};
                    outPath.push_back(curr);
                    found = true;
                    break;
                }
            }

            if (found)
                break;
        }
    }
}
