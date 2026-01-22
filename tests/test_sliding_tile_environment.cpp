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
        outfile << "1 2 3 4 5 6 7 8 9 10 11 12 13 14 0 15\n"; // 1: One move from goal
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
    EXPECT_TRUE(env->is_goal(goal_state));

    uint32_t start_handle = env->get_start_node();
    SlidingTileEnvironment::T not_goal_state = env->get_state(start_handle);
    EXPECT_FALSE(env->is_goal(not_goal_state));
}

TEST_F(SlidingTileEnvironmentTest, HeuristicCalculation) {
    SlidingTileEnvironment::T state;

    // Goal state
    state = 0x123456789ABCDEF0;
    EXPECT_EQ(env->get_heuristic(state), 0);

    // One move away state from fixture
    uint32_t start_handle = env->get_start_node();
    state = env->get_state(start_handle);
    EXPECT_EQ(env->get_heuristic(state), 1);

    // State where 14 and 15 are swapped from goal
    // 1 ... 13, 15, 14, 0
    state = 0x123456789ABCDFE0;
    // Tile 14 (at pos 14) needs to go to pos 13 -> dist 1
    // Tile 15 (at pos 13) needs to go to pos 14 -> dist 1
    EXPECT_EQ(env->get_heuristic(state), 2);
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

    // Expected state when blank moves UP from 14 to 10
    SlidingTileEnvironment::T n_state_up = 0x123456789A0CDEBFULL;
    // Expected state when blank moves LEFT from 14 to 13
    SlidingTileEnvironment::T n_state_left = 0x123456789ABCD0EFULL;
    // Expected state when blank moves RIGHT from 14 to 15 (goal)
    SlidingTileEnvironment::T n_state_right = 0x123456789ABCDEF0ULL;

    const auto& pool = env->get_pool();
    for (uint32_t neighbor_handle : neighbors) {
        SlidingTileEnvironment::T neighbor_state = env->get_state(neighbor_handle);

        if (neighbor_state == n_state_up) found_up = true;
        if (neighbor_state == n_state_left) found_left = true;
        if (neighbor_state == n_state_right) found_right = true;

        const Node& neighbor_node = pool[neighbor_handle];
        EXPECT_EQ(neighbor_node.g, 1);
        EXPECT_EQ(neighbor_node.parent, start_handle);
    }

    EXPECT_TRUE(found_up);
    EXPECT_TRUE(found_left);
    EXPECT_TRUE(found_right);
}

TEST_F(SlidingTileEnvironmentTest, ShorterPathUpdate) {
    uint32_t start_handle = env->get_start_node();
    auto& pool = env->get_pool();

    // Get a neighbor
    std::vector<uint32_t> neighbors;
    env->get_successors(start_handle, neighbors);
    ASSERT_FALSE(neighbors.empty());
    uint32_t neighbor_handle = neighbors[0];

    EXPECT_EQ(pool[neighbor_handle].g, 1);

    // Manually increase the g-value of the neighbor
    pool[neighbor_handle].g = 100;

    // Get successors of start again. It should find the neighbor again
    // and update its g-value back to 1 because 1 < 100.
    std::vector<uint32_t> updated_neighbors;
    env->get_successors(start_handle, updated_neighbors);
    
    EXPECT_EQ(pool[neighbor_handle].g, 1);
    EXPECT_EQ(pool[neighbor_handle].parent, start_handle);

    // Check that the node was returned to be re-processed.
    bool found = false;
    for(auto h : updated_neighbors) {
        if (h == neighbor_handle) {
            found = true;
            break;
        }
    }
    EXPECT_TRUE(found);
}
