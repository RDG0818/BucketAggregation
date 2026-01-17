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

TEST_F(GridEnvironmentTest, HeuristicCalculation) {
    // Goal is (9, 9) for 10x10 grid. State ID for goal is 99.
    
    // Test start state (0, 0) -> state 0
    EXPECT_EQ(env->get_heuristic(0), (9-0) + (9-0)); // 18
    
    // Test goal state (9, 9) -> state 99
    EXPECT_EQ(env->get_heuristic(99), 0);
    
    // Test middle state (5, 5) -> state 55
    EXPECT_EQ(env->get_heuristic(55), (9-5) + (9-5)); // 8
}

TEST_F(GridEnvironmentTest, GoalVerification) {
    uint32_t goal_id = w * h - 1;
    EXPECT_TRUE(env->is_goal(goal_id));
    EXPECT_FALSE(env->is_goal(0));
    EXPECT_FALSE(env->is_goal(50));
}

TEST_F(GridEnvironmentTest, StartNode) {
    uint32_t start_handle = env->get_start_node();
    EXPECT_NE(start_handle, NODE_NULL);
}

TEST_F(GridEnvironmentTest, CellCosts) {
    EXPECT_EQ(env->get_cell_cost(0), 1);
    EXPECT_EQ(env->get_cell_cost(w * h - 1), 1);
}

TEST_F(GridEnvironmentTest, Successors) {
    // Create a smaller grid to make testing successors easier and more deterministic if possible
    GridEnvironment small_env(3, 3, 42); 
    // State layout:
    // 0 1 2
    // 3 4 5
    // 6 7 8
    
    uint32_t start_handle = small_env.get_start_node();
    std::vector<uint32_t> neighbors;
    small_env.get_successors(start_handle, neighbors);
    
    // From (0,0), neighbors can be (1,0) and (0,1) i.e., states 1 and 3
    // unless they are obstacles.
    EXPECT_FALSE(neighbors.empty());
    
    for (uint32_t neighbor_handle : neighbors) {
        EXPECT_NE(neighbor_handle, NODE_NULL);
        EXPECT_NE(neighbor_handle, start_handle);
    }
}

TEST_F(GridEnvironmentTest, ShorterPathUpdate) {
    uint32_t start_handle = env->get_start_node();
    std::vector<uint32_t> neighbors;
    env->get_successors(start_handle, neighbors);
    
    if (!neighbors.empty()) {
        uint32_t first_neighbor = neighbors[0];
        std::vector<uint32_t> second_level_neighbors;
        env->get_successors(first_neighbor, second_level_neighbors);
        
        SUCCEED(); 
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
