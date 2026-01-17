#include <gtest/gtest.h>
#include "environments/environments.h"
#include <vector>
#include <numeric>
#include <algorithm>

class PancakeEnvironmentTest : public ::testing::Test {
protected:
    void SetUp() override {
        // Use a fixed seed for reproducibility and a small capacity for speed
        env = new PancakeEnvironment(12345, 100);
    }

    void TearDown() override {
        delete env;
    }

    PancakeEnvironment* env;
};

TEST_F(PancakeEnvironmentTest, ConstructorState) {
    auto start_state = env->get_state(env->get_start_node());
    
    PancakeEnvironment::T goal_state;
    std::iota(goal_state.begin(), goal_state.end(), 0);

    // The start state should not be the goal state (highly unlikely with the seed)
    EXPECT_NE(start_state, goal_state);

    // The start state should be a permutation of the goal state.
    std::sort(start_state.begin(), start_state.end());
    EXPECT_EQ(start_state, goal_state);
}

TEST_F(PancakeEnvironmentTest, GoalVerification) {
    PancakeEnvironment::T goal_state;
    std::iota(goal_state.begin(), goal_state.end(), 0);
    EXPECT_TRUE(env->is_goal(goal_state));

    auto start_state = env->get_state(env->get_start_node());
    EXPECT_FALSE(env->is_goal(start_state));
}

TEST_F(PancakeEnvironmentTest, HeuristicCalculation) {
    PancakeEnvironment::T state;

    // Test 1: Goal state should have h=0
    std::iota(state.begin(), state.end(), 0);
    EXPECT_EQ(env->get_heuristic(state), 0);

    // Test 2: A reversed state should have h=0 with the current heuristic
    std::iota(state.rbegin(), state.rend(), 0);
    EXPECT_EQ(env->get_heuristic(state), 0);

    // Test 3: A state with a known number of gaps
    // 1, 0, 2, 3, ...
    std::iota(state.begin(), state.end(), 0);
    std::swap(state[0], state[1]); // state is now 1, 0, 2, 3, ...
    // Gaps: abs(0-2)=2. h should be 1.
    EXPECT_EQ(env->get_heuristic(state), 1);
    
    // Test 4: Another state
    // 0, 2, 1, 3, ...
    std::iota(state.begin(), state.end(), 0);
    std::swap(state[1], state[2]); // state is now 0, 2, 1, 3, ...
    // Gaps: abs(0-2)=2, abs(2-1)=1, abs(1-3)=2. h should be 2.
    EXPECT_EQ(env->get_heuristic(state), 2);
}

TEST_F(PancakeEnvironmentTest, StartNode) {
    uint32_t start_handle = env->get_start_node();
    EXPECT_NE(start_handle, NODE_NULL);

    const auto& pool = env->get_pool();
    const Node& start_node = pool[start_handle];
    
    EXPECT_EQ(start_node.g, 0);
    EXPECT_EQ(start_node.parent, NODE_NULL);

    auto start_state = env->get_state(start_handle);
    EXPECT_EQ(start_node.h, env->get_heuristic(start_state));
}


TEST_F(PancakeEnvironmentTest, SuccessorGeneration) {
    uint32_t start_handle = env->get_start_node();
    auto parent_state = env->get_state(start_handle);

    std::vector<uint32_t> neighbors;
    env->get_successors(start_handle, neighbors);

    // There should be N-1 successors for flipping prefixes of length 2 to N
    EXPECT_EQ(neighbors.size(), 47);

    // Check a specific successor
    // Flip the top 2 elements
    PancakeEnvironment::T expected_state = parent_state;
    std::reverse(expected_state.begin(), expected_state.begin() + 2);
    
    bool found = false;
    for (uint32_t neighbor_handle : neighbors) {
        auto neighbor_state = env->get_state(neighbor_handle);
        if (neighbor_state == expected_state) {
            found = true;
            const auto& pool = env->get_pool();
            const Node& neighbor_node = pool[neighbor_handle];
            EXPECT_EQ(neighbor_node.g, 1);
            EXPECT_EQ(neighbor_node.parent, start_handle);
            break;
        }
    }
    EXPECT_TRUE(found);
}
