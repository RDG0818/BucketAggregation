#pragma once

#include "utils/utils.h"
#include "environments/node.h"

#include <vector>
#include <limits>
#include <cstdint>

template <typename E, typename PQ> 
class AnytimeAStar {

public:

  AnytimeAStar(E& env, PQ& priority_queue, utils::SearchStats* stats = nullptr) 
  : env_(env), 
    priority_queue_(priority_queue),
    stats_(stats) {};

  void solve() {
    env_.reset_search();

    auto& pool = env_.get_pool();
    uint32_t start_node = env_.get_start_node();

    uint32_t start_h = env_.get_heuristic(start_node);
    pool.set_g(start_node, 0);

    priority_queue_.push(start_node, 0 + start_h, start_h);
    uint32_t incumbent_cost = INF_COST;

    std::vector<uint32_t> neighbors;
    neighbors.reserve(16);

    while (!priority_queue_.empty()) {
      uint32_t u = priority_queue_.pop();

      if (stats_) stats_->nodes_expanded++;

      uint32_t u_g = pool.get_g(u); // check on staleness
      uint32_t u_h = env_.get_heuristic(u); // Inlined computation
      uint32_t u_f = u_g + u_h;

      if (u_f >= incumbent_cost) continue; 

      if (env_.is_goal(u)) {
        incumbent_cost = u_g;
        if (stats_) { stats_->solution_cost = incumbent_cost; }
        continue;
      }

      env_.get_successors(u, neighbors);

      for (uint32_t v : neighbors) {
        uint32_t cost = env_.get_edge_cost(u, v);
        uint32_t new_g = u_g + cost;

        uint32_t v_h = env_.get_heuristic(v);
        uint32_t new_f = new_g + v_h;
              
        if (new_f>= incumbent_cost) {
          continue;
        }

        if (new_g < pool.get_g(v)) {
          pool.set_g(v, new_g);
          pool.set_parent(v, u);

          if (stats_) stats_->nodes_generated++;
          priority_queue_.push(v, new_f, v_h);
        }
      }
    }
  }

private:

  E& env_;
  PQ& priority_queue_;
  utils::SearchStats* stats_;

};