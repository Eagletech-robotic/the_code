#pragma once

#include <array>
#include <cstdint>
#include <utils/constants.hpp>
#include <utils/sized_array.hpp>

enum class GridSquare : uint8_t {
    Empty,
    Target,
    Obstacle,
};

struct Coord {
    uint8_t x;
    uint8_t y;
};

void compute_shortest_path(const std::array<std::array<GridSquare, FIELD_HEIGHT_SQ>, FIELD_WIDTH_SQ> &grid, Coord root,
                           SizedArray<Coord, FIELD_WIDTH_SQ * FIELD_HEIGHT_SQ> &outPath);
