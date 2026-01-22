#include <gtest/gtest.h>
#include "algorithms/a_star.h"
#include "algorithms/ana_star.h"
#include "queues/binary_heap.h"
#include "environments/environments.h"

// This test suite verifies that ANA* (run to completion) finds the same 
// optimal solution as A* on a deterministic environment.
TEST(ANAStarTest, FindsOptimalSolution) {
    const uint32_t width = 20;
    const uint32_t height = 20;
    const uint32_t seed = 123; // Use a fixed seed for a deterministic grid

    double a_star_cost = -1.0;
    double ana_star_cost = -1.0;

    // Step 1: Run A* to get the ground truth optimal cost
    {
        GridEnvironment env(width, height, seed);
        using AStarHeap = BinaryHeap<uint32_t, std::less<uint32_t>>;
        AStarHeap heap(env.get_pool());
        AStar<GridEnvironment, AStarHeap> solver(env, heap);

        solver.solve();

        uint32_t goal_handle = env.get_node_handle(width * height - 1);
        if (goal_handle != NODE_NULL) {
            a_star_cost = env.get_pool()[goal_handle].g;
        }
    }

    // Ensure A* actually found a path
    ASSERT_NE(a_star_cost, -1.0) << "A* failed to find a solution, cannot verify ANA*.";

    // Step 2: Run ANA* on the same environment
    {
        GridEnvironment env(width, height, seed);
        using ANAStarHeap = BinaryHeap<double, std::greater<double>>;
        ANAStarHeap heap(env.get_pool());
        ANAStar<GridEnvironment, ANAStarHeap> solver(env, heap);

        solver.solve();

        uint32_t goal_handle = env.get_node_handle(width * height - 1);
        if (goal_handle != NODE_NULL) {
            ana_star_cost = env.get_pool()[goal_handle].g;
        }
    }
    
    // Step 3: Assert that ANA*, when run to completion, finds the same optimal cost.
    ASSERT_EQ(ana_star_cost, a_star_cost);
}
