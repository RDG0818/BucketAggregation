#include <gtest/gtest.h>
#include "algorithms/anytime_a_star.h"
#include "queues/binary_heap.h"
#include "environments/node.h"
#include <vector>
#include <unordered_map>
#include <cmath>
#include <algorithm>

class TestEnvironment {
public:
  using T = uint32_t;

  TestEnvironment(T start, T goal) : start_state_(start), goal_state_(goal) {
    pool_.resize_state_space(100);
  }
  
  void reset_search() {
    pool_.prepare_for_search();
  }

  void add_edge(T u, T v, uint32_t cost) {
    adj_[u].push_back({v, cost});
  }

  uint32_t get_start_node() { return start_state_; }

  void get_successors(uint32_t u, std::vector<uint32_t>& neighbors) {
    neighbors.clear();
    if (adj_.find(u) == adj_.end()) return;
    
    for (const auto& edge : adj_[u]) {
      neighbors.push_back(edge.first);
    }
  }

  uint32_t get_edge_cost(uint32_t u, uint32_t v) const {
    if (adj_.find(u) != adj_.end()) {
      for (const auto& edge : adj_.at(u)) {
        if (edge.first == v) return edge.second;
      }
    }
    return 1;
  }

  uint32_t get_heuristic(uint32_t u) const {
    return std::abs(static_cast<int>(u) - static_cast<int>(goal_state_));
  }

  bool is_goal(uint32_t u) const { return u == goal_state_; }

  NodePool& get_pool() { return pool_; }
  const NodePool& get_pool() const { return pool_; }

private:
  T start_state_;
  T goal_state_;
  NodePool pool_;
  std::unordered_map<T, std::vector<std::pair<T, uint32_t>>> adj_;
};

class AnytimeAStarTest : public ::testing::Test {};

TEST_F(AnytimeAStarTest, StraightLinePath) {
  TestEnvironment env(0, 4);
  env.add_edge(0, 1, 1);
  env.add_edge(1, 2, 1);
  env.add_edge(2, 3, 1);
  env.add_edge(3, 4, 1);

  BinaryHeap<uint32_t, std::greater<uint32_t>> pq;
  AnytimeAStar<TestEnvironment, BinaryHeap<uint32_t, std::greater<uint32_t>>> anytime_a_star(env, pq);
  anytime_a_star.solve();

  const auto& pool = env.get_pool();
  EXPECT_EQ(pool.get_g(4), 4);
  
  // Reconstruct
  std::vector<uint32_t> path;
  uint32_t curr = 4;
  while (curr != 0 && curr != NODE_NULL) {
      path.push_back(curr);
      curr = pool.get_parent(curr);
  }
  path.push_back(0);
  std::reverse(path.begin(), path.end());

  std::vector<uint32_t> expected = {0, 1, 2, 3, 4};
  EXPECT_EQ(path, expected);
}

TEST_F(AnytimeAStarTest, PathWithChoice) {
  TestEnvironment env(0, 3);
  // Suboptimal path (Cost 6)
  env.add_edge(0, 1, 1);
  env.add_edge(1, 3, 5);
  
  // Optimal path (Cost 2)
  env.add_edge(0, 2, 1);
  env.add_edge(2, 3, 1);

  BinaryHeap<uint32_t, std::greater<uint32_t>> pq;
  AnytimeAStar<TestEnvironment, BinaryHeap<uint32_t, std::greater<uint32_t>>> anytime_a_star(env, pq);
  anytime_a_star.solve();
  
  const auto& pool = env.get_pool();
  EXPECT_EQ(pool.get_g(3), 2);

  std::vector<uint32_t> path;
  uint32_t curr = 3;
  while (curr != 0 && curr != NODE_NULL) {
      path.push_back(curr);
      curr = pool.get_parent(curr);
  }
  path.push_back(0);
  std::reverse(path.begin(), path.end());
  
  std::vector<uint32_t> expected = {0, 2, 3};
  EXPECT_EQ(path, expected);
}

TEST_F(AnytimeAStarTest, PruningLogic) {
  TestEnvironment env(0, 4);
  
  // Path A: 0 -> 1 -> 4 (Cost 10)
  // Heuristic will guide towards this if H is misleading (not possible in this simple env, but topology works)
  env.add_edge(0, 1, 1);
  env.add_edge(1, 4, 9); 
  
  // Path B: 0 -> 2 -> 3 -> 4 (Cost 3)
  env.add_edge(0, 2, 1);
  env.add_edge(2, 3, 1);
  env.add_edge(3, 4, 1);
  
  BinaryHeap<uint32_t, std::greater<uint32_t>> pq;
  AnytimeAStar<TestEnvironment, BinaryHeap<uint32_t, std::greater<uint32_t>>> anytime_a_star(env, pq);
  anytime_a_star.solve();
  
  const auto& pool = env.get_pool();
  EXPECT_EQ(pool.get_g(4), 3);
}