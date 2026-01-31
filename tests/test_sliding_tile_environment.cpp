#include <gtest/gtest.h>
#include "environments/environments.h"
#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <cstdio> // For std::remove

class SlidingTileEnvironmentTest : public ::testing::Test {
protected:
  void SetUp() override {
    // Create a test instance file
    std::ofstream outfile("test_tile_instances.txt");
    outfile << "1 2 3 4 5 6 7 8 9 10 11 12 13 14 15 0\n"; // 0: Goal state
    outfile << "1 2 3 4 5 6 7 8 9 10 11 12 13 14 0 15\n"; // 1: One move from goal (Blank at 14)
    outfile << "1 2 3 4 5 6 7 8 9 10 11 0 13 14 15 12\n"; // 2: blank at index 11
    outfile.close();

    // Use the second instance (index 1) for most tests
    env = new SlidingTileEnvironment(1, "test_tile_instances.txt", 1000);
  }

  void TearDown() override {
    delete env;
    std::remove("test_tile_instances.txt");
  }

  SlidingTileEnvironment* env;
};


TEST_F(SlidingTileEnvironmentTest, GetTile) {
  SlidingTileEnvironment::T goal_state = 0x123456789ABCDEF0;
  EXPECT_EQ(env->get_tile(goal_state, 0), 1);
  EXPECT_EQ(env->get_tile(goal_state, 1), 2);
  EXPECT_EQ(env->get_tile(goal_state, 14), 15);
  EXPECT_EQ(env->get_tile(goal_state, 15), 0);

  SlidingTileEnvironment::T test_state = 0x0FEDCBA987654321;
  EXPECT_EQ(env->get_tile(test_state, 0), 0);
  EXPECT_EQ(env->get_tile(test_state, 1), 15);
  EXPECT_EQ(env->get_tile(test_state, 15), 1);
}

TEST_F(SlidingTileEnvironmentTest, ConstructorAndStartState) {
  uint32_t start_handle = env->get_start_node();
  SlidingTileEnvironment::T start_state = env->get_state(start_handle);
  
  // Instance 1: "1 2 3 4 5 6 7 8 9 10 11 12 13 14 0 15"
  // Packed: 1, 2, 3, 4, 5, 6, 7, 8, 9, A, B, C, D, E, 0, F
  SlidingTileEnvironment::T expected_start_state = 0x123456789ABCDE0FULL;
  EXPECT_EQ(start_state, expected_start_state);

  // Test with goal state instance (index 0)
  SlidingTileEnvironment goal_env(0, "test_tile_instances.txt", 1000);
  uint32_t goal_start_handle = goal_env.get_start_node();
  SlidingTileEnvironment::T goal_start_state = goal_env.get_state(goal_start_handle);
  SlidingTileEnvironment::T expected_goal_state = 0x123456789ABCDEF0;

  EXPECT_EQ(goal_start_state, expected_goal_state);
}

TEST_F(SlidingTileEnvironmentTest, GoalVerification) {
  SlidingTileEnvironment::T goal_state = 0x123456789ABCDEF0;
  
  // We must ensure the state exists in the system to get a valid ID
  uint32_t goal_id = env->get_or_create_id(goal_state);
  EXPECT_TRUE(env->is_goal(goal_id));

  uint32_t start_handle = env->get_start_node();
  EXPECT_FALSE(env->is_goal(start_handle));
}

TEST_F(SlidingTileEnvironmentTest, HeuristicCalculation) {
  SlidingTileEnvironment::T state;

  // Goal state
  state = 0x0123456789ABCDEF;
  uint32_t goal_id = env->get_or_create_id(state);
  EXPECT_EQ(env->get_heuristic(goal_id), 0);

  // One move away state (Start Node from fixture)
  uint32_t start_handle = env->get_start_node();
  EXPECT_EQ(env->get_heuristic(start_handle), 1);

  // State where 14 and 15 are swapped from goal
  // 1 ... 13, 15, 14, 0
  state = 0x123456789ABCDFE0;
  // Tile 14 (at pos 14) needs to go to pos 13 -> dist 1
  // Tile 15 (at pos 13) needs to go to pos 14 -> dist 1
  uint32_t swapped_id = env->get_or_create_id(state);
  EXPECT_EQ(env->get_heuristic(swapped_id), 2);
}

TEST_F(SlidingTileEnvironmentTest, SuccessorGeneration) {
  uint32_t start_handle = env->get_start_node();
  
  std::vector<uint32_t> neighbors;
  env->get_successors(start_handle, neighbors);

  // Blank is at index 14. Can move to 10 (up), 13 (left), 15 (right).
  EXPECT_EQ(neighbors.size(), 3);

  bool found_up = false;
  bool found_left = false;
  bool found_right = false;

  // Expected state when blank moves UP from 14 to 10 (Swap 0 and B)
  SlidingTileEnvironment::T n_state_up = 0x123456789A0CDEBFULL;
  // Expected state when blank moves LEFT from 14 to 13 (Swap 0 and E)
  SlidingTileEnvironment::T n_state_left = 0x123456789ABCD0EFULL;
  // Expected state when blank moves RIGHT from 14 to 15 (Swap 0 and F - Goal)
  SlidingTileEnvironment::T n_state_right = 0x123456789ABCDEF0ULL;

  for (uint32_t neighbor_handle : neighbors) {
    SlidingTileEnvironment::T neighbor_state = env->get_state(neighbor_handle);

    if (neighbor_state == n_state_up) found_up = true;
    if (neighbor_state == n_state_left) found_left = true;
    if (neighbor_state == n_state_right) found_right = true;
    }

    EXPECT_TRUE(found_up);
    EXPECT_TRUE(found_left);
    EXPECT_TRUE(found_right);
}

TEST_F(SlidingTileEnvironmentTest, GraphConsistency) {
  uint32_t start_handle = env->get_start_node();
  std::vector<uint32_t> neighbors;
  env->get_successors(start_handle, neighbors);
  
  ASSERT_FALSE(neighbors.empty());
  uint32_t neighbor_id = neighbors[0];
  SlidingTileEnvironment::T neighbor_state = env->get_state(neighbor_id);

  uint32_t duplicate_id = env->get_or_create_id(neighbor_state);

  EXPECT_EQ(neighbor_id, duplicate_id);
}