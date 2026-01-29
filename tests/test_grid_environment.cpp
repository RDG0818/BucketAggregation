#include <gtest/gtest.h>
#include "environments/environments.h"
#include <vector>

class GridEnvironmentTest : public ::testing::Test {
protected:
  void SetUp() override {
    w = 10;
    h = 10;
    env = new GridEnvironment(w, h, 1234); 
  }

  void TearDown() override {
    delete env;
  }

  uint32_t w, h;
  GridEnvironment* env;
};

TEST_F(GridEnvironmentTest, PoolInitialization) {
  EXPECT_EQ(env->get_pool().size(), w * h);
  EXPECT_NO_THROW(env->get_pool().get_g(w * h - 1));
}

TEST_F(GridEnvironmentTest, HeuristicCalculation) {
  // Goal is (9, 9) for 10x10 grid. State ID for goal is 99.
  
  EXPECT_EQ(env->get_heuristic(0), (9-0) + (9-0)); // 18
  EXPECT_EQ(env->get_heuristic(99), 0);
  EXPECT_EQ(env->get_heuristic(55), (9-5) + (9-5)); // 8
}

TEST_F(GridEnvironmentTest, GoalVerification) {
  uint32_t goal_id = w * h - 1;
  EXPECT_TRUE(env->is_goal(goal_id));
  EXPECT_FALSE(env->is_goal(0));
  EXPECT_FALSE(env->is_goal(50));
}

TEST_F(GridEnvironmentTest, EdgeCosts) {
  EXPECT_EQ(env->get_edge_cost(1, 0), 1);
  EXPECT_EQ(env->get_edge_cost(w * h - 2, w * h - 1), 1);
}

TEST_F(GridEnvironmentTest, Successors) {
  // Create a smaller grid to make testing successors easier and more deterministic if possible
  GridEnvironment small_env(3, 3, 42); 
  // State layout:
  // 0 1 2
  // 3 4 5
  // 6 7 8
  
  uint32_t center_node = 0;
  std::vector<uint32_t> neighbors;
  small_env.get_successors(center_node, neighbors);
  
  EXPECT_FALSE(neighbors.empty());
  
  for (uint32_t neighbor_id : neighbors) {
    EXPECT_LT(neighbor_id, 9);
    EXPECT_NE(neighbor_id, center_node);
    uint32_t diff = (neighbor_id > center_node) ? (neighbor_id - center_node) : (center_node - neighbor_id);
    bool is_adjacent = (diff == 1) || (diff == 3);
    EXPECT_TRUE(is_adjacent) << "Neighbor " << neighbor_id << " is not geometrically adjacent to " << center_node;
  }
}

TEST_F(GridEnvironmentTest, HeuristicAdmissibility) {
  for (uint32_t i = 0; i < w * h; ++i) {
    uint32_t h_val = env->get_heuristic(i);
    uint32_t dx = (w - 1) - (i % w);
    uint32_t dy = (h - 1) - (i / w);
    EXPECT_EQ(h_val, dx + dy);
  }
}
