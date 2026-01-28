#pragma once

#include <vector>
#include <cstdint>
#include "utils/utils.h"

template <typename E, typename PQ> 
class AStar {

public:

  AStar(E& env, PQ& priority_queue, utils::SearchStats* stats = nullptr) 
    : env_(env), priority_queue_(priority_queue), stats_(stats) {};

  void solve() {
    env_.reset_search();
    auto& pool = env_.get_pool();
    uint32_t start_node = env_.get_start_node();
    uint32_t start_h = env_.get_heuristic(start_node);

    pool.set_g(start_node, 0);
    priority_queue_.push(start_node, 0 + start_h, start_h);

    std::vector<uint32_t> neighbors;
    neighbors.reserve(16);

    while (!priority_queue_.empty()) {
      uint32_t u = priority_queue_.pop();
      if (stats_) { stats_->nodes_expanded++; }

      if (env_.is_goal(u)) {
        // Can construct path if needed
        return;
      }

      env_.get_successors(u, neighbors);
      uint32_t u_g = pool.get_g(u);

      for (uint32_t v : neighbors) {
        uint32_t cost = env_.get_edge_cost(u, v);
        uint32_t new_g = u_g + cost;

        if (new_g < pool.get_g(v)) {
          pool.set_g(v, new_g);
          pool.set_parent(v, u);
          uint32_t h = env_.get_heuristic(v);
          priority_queue_.push(v, new_g + h, h);

          if (stats_) stats_->nodes_generated++;
        }
      }
    }
  }

private:

  E& env_;
  PQ& priority_queue_;
  utils::SearchStats* stats_;

};