#pragma once

#include <vector>
#include <limits>
#include <cstdint>

template <typename E, typename PQ> 
class AWAStar {

public:

  AWAStar(E& env, PQ& priority_queue) : env_(env), priority_queue_(priority_queue) {};

  void solve() {
    uint32_t start_node = env_.get_start_node();
    const auto& start_node_data = env_.get_pool()[start_node];
    priority_queue_.push(start_node, start_node_data.g + start_node_data.h);

    uint32_t incumbent_cost = std::numeric_limits<uint32_t>::max();
    std::vector<uint32_t> neighbors;

    while (!priority_queue_.empty()) {
      uint32_t current_node = priority_queue_.pop();
      const auto& current_node_data = env_.get_pool()[current_node];

      if (current_node_data.g + current_node_data.h >= incumbent_cost) {
        continue;
      }

      if (env_.is_goal(env_.get_state(current_node))) {
        incumbent_cost = current_node_data.g;
        continue;
      }

      neighbors.clear();
      env_.get_successors(current_node, neighbors);

      for (uint32_t neighbor : neighbors) {
        const auto& neighbor_data = env_.get_pool()[neighbor];
        uint32_t priority = neighbor_data.g + neighbor_data.h;
              
        if (priority >= incumbent_cost) {
          continue;
        }

        if (priority_queue_.contains(neighbor)) {
          priority_queue_.decrease_key(neighbor, priority);
        } else {
          priority_queue_.push(neighbor, priority);
        }
      }
    }
  }

private:

  E& env_;
  PQ& priority_queue_;

};
