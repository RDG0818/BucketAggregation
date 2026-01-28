#include <gtest/gtest.h>
#include "environments/environments.h"
#include <vector>
#include <numeric>
#include <algorithm>

class PancakeEnvironmentTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Seed 12345, Capacity 100
    env = new PancakeEnvironment(12345, 100); 
  }

  void TearDown() override {
    delete env;
  }

  PancakeEnvironment* env;
};

TEST_F(PancakeEnvironmentTest, ConstructorState) {
  uint32_t start_id = env->generate_start_node();
  
  auto start_state = env->get_state(start_id);
  
  PancakeEnvironment::T goal_state;
  std::iota(goal_state.begin(), goal_state.end(), 0);

  EXPECT_NE(start_state, goal_state);

  std::sort(start_state.begin(), start_state.end());
  EXPECT_EQ(start_state, goal_state);
}

TEST_F(PancakeEnvironmentTest, GoalVerification) {
  PancakeEnvironment::T goal_state_array;
  std::iota(goal_state_array.begin(), goal_state_array.end(), 0);

  uint32_t goal_id = env->get_or_create_id(goal_state_array);

  EXPECT_TRUE(env->is_goal(goal_id));

  uint32_t start_id = env->generate_start_node();
  EXPECT_FALSE(env->is_goal(start_id));
}

TEST_F(PancakeEnvironmentTest, HeuristicCalculation) {
  PancakeEnvironment::T state_array;

  std::iota(state_array.begin(), state_array.end(), 0);
  uint32_t id_sorted = env->get_or_create_id(state_array);
  EXPECT_EQ(env->get_heuristic(id_sorted), 0);

  std::iota(state_array.rbegin(), state_array.rend(), 0);
  uint32_t id_reverse = env->get_or_create_id(state_array);
  EXPECT_EQ(env->get_heuristic(id_reverse), 0);

  std::iota(state_array.begin(), state_array.end(), 0);
  std::swap(state_array[0], state_array[1]); 
  uint32_t id_one_gap = env->get_or_create_id(state_array);
  EXPECT_EQ(env->get_heuristic(id_one_gap), 1);

  std::iota(state_array.begin(), state_array.end(), 0);
  std::swap(state_array[1], state_array[2]); 
  uint32_t id_two_gaps = env->get_or_create_id(state_array);
  EXPECT_EQ(env->get_heuristic(id_two_gaps), 2);
}

TEST_F(PancakeEnvironmentTest, StartNode) {
  uint32_t start_id = env->generate_start_node();
  EXPECT_NE(start_id, NODE_NULL);

  const auto& pool = env->get_pool();
  EXPECT_EQ(pool.get_g(start_id), INF_COST); // Should be unvisited
  
  uint32_t calculated_h = env->get_heuristic(start_id);
  EXPECT_GT(calculated_h, 0); 
}

TEST_F(PancakeEnvironmentTest, SuccessorGeneration) {
  uint32_t start_id = env->generate_start_node();
  auto parent_state = env->get_state(start_id);

  std::vector<uint32_t> neighbors;
  env->get_successors(start_id, neighbors);

  // There should be N-1 successors (flipping 2..N)
  // PancakeEnvironment::N is 48
  EXPECT_EQ(neighbors.size(), 47);

  // Verify a specific successor exists: Flipping top 2 elements
  PancakeEnvironment::T expected_state = parent_state;
  std::reverse(expected_state.begin(), expected_state.begin() + 2);
  
  bool found = false;
  for (uint32_t neighbor_id : neighbors) {
    auto neighbor_state = env->get_state(neighbor_id);
    
    if (neighbor_state == expected_state) {
      found = true;
      // Verify ID consistency
      // The environment creates IDs lazily, so this neighbor must exist in pool
      EXPECT_NE(neighbor_id, NODE_NULL);
      EXPECT_NE(neighbor_id, start_id);
      break;
    }
  }
  EXPECT_TRUE(found) << "The successor corresponding to flipping top 2 pancakes was not found.";
}