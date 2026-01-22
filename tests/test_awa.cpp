#include <gtest/gtest.h>
#include "algorithms/awa.h"
#include "queues/binary_heap.h"
#include "environments/node.h"
#include <vector>
#include <map>
#include <unordered_map>
#include <cmath>
#include <algorithm>

class TestEnvironment {
public:
    using T = uint32_t;

    NodePool pool_;
    std::map<T, std::vector<std::pair<T, uint32_t>>> edges; // state -> (neighbor_state, cost)
    std::vector<T> node_states_; // node_handle -> state_id
    std::unordered_map<T, uint32_t> lookup_table_; // state_id -> node_handle
    T start_state_;
    T goal_state_;

    TestEnvironment(T start, T goal) : start_state_(start), goal_state_(goal) {
        pool_.reserve(100);
        node_states_.reserve(100);
    }
    
    void add_edge(T u, T v, uint32_t cost) {
        edges[u].push_back({v, cost});
    }

    uint32_t get_heuristic(T state_id) const {
        return std::abs(static_cast<int>(state_id) - static_cast<int>(goal_state_));
    }

    bool is_goal(T state_id) const { return state_id == goal_state_; }

    T get_state(uint32_t node_handle) const { return node_states_[node_handle]; }

    NodePool& get_pool() { return pool_; }
    const NodePool& get_pool() const { return pool_; }

    uint32_t get_start_node() {
        uint32_t handle = pool_.allocate(NODE_NULL, 0, get_heuristic(start_state_));
        node_states_.push_back(start_state_);
        lookup_table_[start_state_] = handle;
        return handle;
    }
    
    uint32_t get_node_handle(T state_id) const {
        auto it = lookup_table_.find(state_id);
        if (it != lookup_table_.end()) {
            return it->second;
        }
        return NODE_NULL;
    }

    void get_successors(uint32_t parent_handle, std::vector<uint32_t>& neighbors) {
        const uint32_t parent_g = pool_[parent_handle].g;
        const T parent_state_id = node_states_[parent_handle];

        if (edges.find(parent_state_id) == edges.end()) return;

        for (auto const& edge : edges[parent_state_id]) {
            T n_state_id = edge.first;
            uint32_t move_cost = edge.second;
            uint32_t new_g = parent_g + move_cost;
            uint32_t n_handle = get_node_handle(n_state_id);
            
            if (n_handle == NODE_NULL) {
                uint32_t h_val = get_heuristic(n_state_id);
                n_handle = pool_.allocate(parent_handle, new_g, h_val);
                lookup_table_[n_state_id] = n_handle;
                node_states_.push_back(n_state_id);
                neighbors.push_back(n_handle);
            } else if (new_g < pool_[n_handle].g) {
                pool_[n_handle].g = new_g;
                pool_[n_handle].parent = parent_handle;
                neighbors.push_back(n_handle);
            }
        }
    }
};

class AWAStarTest : public ::testing::Test {};

TEST_F(AWAStarTest, StraightLinePath) {
    TestEnvironment env(0, 4);
    env.add_edge(0, 1, 1);
    env.add_edge(1, 2, 1);
    env.add_edge(2, 3, 1);
    env.add_edge(3, 4, 1);

    BinaryHeap<int> pq(env.get_pool());
    AWAStar<TestEnvironment, BinaryHeap<int>> awa_star(env, pq);
    awa_star.solve();

    uint32_t goal_handle = env.get_node_handle(4);
    ASSERT_NE(goal_handle, NODE_NULL);
    EXPECT_EQ(env.get_pool()[goal_handle].g, 4);

    std::vector<uint32_t> path;
    uint32_t current = goal_handle;
    while(current != NODE_NULL) {
        path.push_back(env.get_state(current));
        current = env.get_pool()[current].parent;
    }
    std::reverse(path.begin(), path.end());

    std::vector<uint32_t> expected_path = {0, 1, 2, 3, 4};
    EXPECT_EQ(path, expected_path);
}

TEST_F(AWAStarTest, PathWithChoice) {
    TestEnvironment env(0, 3);
    env.add_edge(0, 1, 1); 
    env.add_edge(1, 3, 3); 
    env.add_edge(0, 2, 2); 
    env.add_edge(2, 3, 2); 

    BinaryHeap<int> pq(env.get_pool());
    AWAStar<TestEnvironment, BinaryHeap<int>> awa_star(env, pq);
    awa_star.solve();
    
    uint32_t goal_handle = env.get_node_handle(3);
    ASSERT_NE(goal_handle, NODE_NULL);
    
    EXPECT_EQ(env.get_pool()[goal_handle].g, 4);

    std::vector<uint32_t> path;
    uint32_t current = goal_handle;
    while(current != NODE_NULL) {
        path.push_back(env.get_state(current));
        current = env.get_pool()[current].parent;
    }
    std::reverse(path.begin(), path.end());
    
    std::vector<uint32_t> expected_path = {0, 1, 3};
    EXPECT_EQ(path, expected_path);
}


TEST_F(AWAStarTest, Pruning) {
    TestEnvironment env(0, 4); 
    env.add_edge(0, 1, 3);
    env.add_edge(1, 4, 3); // Path 0-1-4, cost 6. 
    
    env.add_edge(0, 2, 1);
    env.add_edge(2, 3, 1);
    env.add_edge(3, 4, 1); // Path 0-2-3-4, cost 3
    
    BinaryHeap<int> pq(env.get_pool());
    AWAStar<TestEnvironment, BinaryHeap<int>> awa_star(env, pq);
    awa_star.solve();
    
    uint32_t goal_handle = env.get_node_handle(4);
    ASSERT_NE(goal_handle, NODE_NULL);
    
    EXPECT_EQ(env.get_pool()[goal_handle].g, 3);
    
    std::vector<uint32_t> path;
    uint32_t current = goal_handle;
    while(current != NODE_NULL) {
        path.push_back(env.get_state(current));
        current = env.get_pool()[current].parent;
    }
    std::reverse(path.begin(), path.end());

    std::vector<uint32_t> expected_path = {0, 2, 3, 4};
    EXPECT_EQ(path, expected_path);
}
