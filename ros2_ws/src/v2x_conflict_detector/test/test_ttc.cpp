#include <gtest/gtest.h>
#include "v2x_conflict_detector/ttc.hpp"

using namespace v2x;

TEST(Ttc, HeadOnCollision)
{
    Agent a{1, 0, 0, 10, 0, 1.0};
    Agent b{2, 40, 0, -10, 0, 1.0};
    auto t = timeToCollision(a, b);
    ASSERT_TRUE(t.has_value());
    // Gap 40 m, closing 20 m/s, circles touch at 2 m separation -> (40-2)/20 = 1.9 s
    EXPECT_NEAR(*t, 1.9, 1e-6);
}

TEST(Ttc, DivergingVehiclesNeverCollide)
{
    Agent a{1, 0, 0, -5, 0, 1.0};
    Agent b{2, 20, 0, 5, 0, 1.0};
    EXPECT_FALSE(timeToCollision(a, b).has_value());
}

TEST(Ttc, ParallelSameVelocityNeverCollide)
{
    Agent a{1, 0, 0, 8, 0, 1.0};
    Agent b{2, 0, 10, 8, 0, 1.0};
    EXPECT_FALSE(timeToCollision(a, b).has_value());
}

TEST(Ttc, PerpendicularIntersectionCrossing)
{
    Agent a{1, -20, 0, 10, 0, 1.0};   // eastbound
    Agent b{2, 0, -20, 0, 10, 1.0};   // northbound, both reach origin at t=2
    auto t = timeToCollision(a, b);
    ASSERT_TRUE(t.has_value());
    EXPECT_LT(*t, 2.0);
    EXPECT_GT(*t, 1.5);
}

TEST(Ttc, AlreadyOverlappingIsZero)
{
    Agent a{1, 0, 0, 0, 0, 2.0};
    Agent b{2, 1, 0, 0, 0, 2.0};
    auto t = timeToCollision(a, b);
    ASSERT_TRUE(t.has_value());
    EXPECT_DOUBLE_EQ(*t, 0.0);
}

TEST(DetectConflicts, SortsByUrgency)
{
    std::vector<Agent> agents = {
        {1, 0, 0, 10, 0, 1.0},
        {2, 40, 0, -10, 0, 1.0},   // TTC 1.9 with #1
        {3, 10, 0, 10, 0, 1.0},    // ahead of #1, same speed: no conflict with #1
        {4, 14, 0, 0, 0, 1.0},     // stopped: #3 hits it fast
    };
    auto conflicts = detectConflicts(agents, 2.0);
    ASSERT_GE(conflicts.size(), 2u);
    EXPECT_LE(conflicts.front().ttc, conflicts.back().ttc);
}

TEST(DetectConflicts, ThresholdFilters)
{
    std::vector<Agent> agents = {
        {1, 0, 0, 1, 0, 1.0},
        {2, 100, 0, -1, 0, 1.0},   // TTC = 49 s
    };
    EXPECT_TRUE(detectConflicts(agents, 2.0).empty());
    EXPECT_EQ(detectConflicts(agents, 60.0).size(), 1u);
}
