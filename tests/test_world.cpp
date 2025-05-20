#include "eaglesteward/world.hpp"
#include <gtest/gtest.h>
#include <cfloat>

constexpr float kSquare = 0.04f; // SQUARE_SIZE_M
constexpr float kStraight = kSquare;
constexpr float kDiag = kSquare * 1.414f;

class DijkstraTest : public ::testing::Test {
protected:
    World world{RobotColour::Blue};

    void SetUp() override {
        /* clear everything manually – we want a blank field */
        for (auto &row: world.obstacles_field_)
            std::fill(row.begin(), row.end(), ObstacleType::None);

        for (auto &row: world.potential_calculating())
            std::fill(row.begin(), row.end(), FLT_MAX);

        world.pqueue_.clear();
        world.ready_field_ = 1; // ensure potential_calculating() → field[0]

        /* seed a single start cell */
        const int i = 25, j = 25; // ≈ (1 m, 1 m)
        world.potential_calculating()[i][j] = 0.0f;
        world.pqueue_.emplace(0.0f, i, j);
    }
};

TEST_F(DijkstraTest, StraightAndDiagonalCostsAreCorrect) {
    /* run to completion */
    while (world.partial_compute_dijkstra([] { return true; }));

    const auto &pot = world.potential_ready();
    const int i = 25, j = 25;

    EXPECT_FLOAT_EQ(pot[i][j], 0.0f); // start
    EXPECT_FLOAT_EQ(pot[i + 1][j], kStraight); // straight neighbour
    EXPECT_NEAR(pot[i + 1][j + 1], kDiag, 1e-5f); // diagonal neighbour
}
