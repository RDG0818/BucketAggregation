#include <gtest/gtest.h>
#include "algorithms/a_star.h"
#include "algorithms/ana_star.h"
#include "queues/binary_heap.h"
#include "environments/environments.h"

TEST(ANAStarTest, FindsOptimalSolution) {
  const uint32_t width = 20;
  const uint32_t height = 20;
  const uint32_t seed = 123; 
  const uint32_t goal_id = width * height - 1;

  uint32_t a_star_cost = INF_COST;
  uint32_t ana_star_cost = INF_COST;

  {
    GridEnvironment env(width, height, seed);
    
    using AStarHeap = BinaryHeap<uint32_t, std::greater<uint32_t>>;
    AStarHeap heap;
    
    AStar<GridEnvironment, AStarHeap> solver(env, heap);
    solver.solve();

    a_star_cost = env.get_pool().get_g(goal_id);
  }

  ASSERT_NE(a_star_cost, INF_COST) << "A* failed to find a solution on this seed.";

  {
    GridEnvironment env(width, height, seed);
    
    using ANAStarHeap = BinaryHeap<double, std::less<double>>;
    ANAStarHeap heap;
    
    ANAStar<GridEnvironment, ANAStarHeap> solver(env, heap);
    solver.solve();

    ana_star_cost = env.get_pool().get_g(goal_id);
  }
  ASSERT_EQ(ana_star_cost, a_star_cost);
}