#pragma once

#include <vector>
#include <cstdint>
#include <limits>
#include "utils/utils.h"

template <typename E, typename PQ>
class ANAStar {

public:
  ANAStar(E& env, PQ& priority_queue, utils::SearchStats* stats = nullptr) 
    : env_(env), priority_queue_(priority_queue), stats_(stats), G_upper_(std::numeric_limits<double>::max()) {};

        
void solve() {
  env_.reset_search();
  auto& pool = env_.get_pool();
  uint32_t start_node = env_.get_start_node();

  uint32_t start_h = env_.get_heuristic(start_node);
  pool.set_g(start_node, 0);

  priority_queue_.push(start_node, calculate_e(0, start_h), start_h);

  std::vector<uint32_t> neighbors;
  neighbors.reserve(16);

  while (!priority_queue_.empty()) {

    uint32_t u = priority_queue_.pop();

    if (stats_) stats_->nodes_expanded++;

    uint32_t u_g = pool.get_g(u);
    uint32_t u_h = env_.get_heuristic(u);

    if (static_cast<double>(u_g) + u_h >= G_upper_) continue; 
    

    if (env_.is_goal(u)) {
      if (u_g < G_upper_) {
        G_upper_ = static_cast<double>(u_g);
        if (stats_) stats_->solution_cost = G_upper_;

          auto calculator = [&](uint32_t id) -> double {
            return calculate_e(pool.get_g(id), env_.get_heuristic(id));
          };
          priority_queue_.rebuild(calculator);
        }
      continue;
    }

    env_.get_successors(u, neighbors);

    for (uint32_t v : neighbors) {
      uint32_t cost = env_.get_edge_cost(u, v);
      uint32_t new_g = u_g + cost;
      uint32_t v_h = env_.get_heuristic(v);

      if (static_cast<double>(new_g) + v_h >= G_upper_) {
        continue;
      }

      if (new_g < pool.get_g(v)) {
        pool.set_g(v, new_g);
        pool.set_parent(v, u);
                    
        if (stats_) stats_->nodes_generated++;
                    
        double e_val = calculate_e(new_g, v_h);
        priority_queue_.push(v, e_val, v_h);
      }
    }
  }
}

private:

  double calculate_e(uint32_t g, uint32_t h) const {
    if (h == 0) {
      return std::numeric_limits<double>::max(); // Max priority for goals
    }
    if (g >= G_upper_) {
      return std::numeric_limits<double>::lowest(); // Pruned
    }
    return (G_upper_ - static_cast<double>(g)) / static_cast<double>(h);
  }

  E& env_;
  PQ& priority_queue_;
  utils::SearchStats* stats_;
  double G_upper_;

};
