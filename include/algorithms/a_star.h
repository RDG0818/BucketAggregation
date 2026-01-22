#pragma once

#include <vector>
#include <cstdint>

template <typename E, typename PQ> 
class AStar {

public:

  AStar(E& env, PQ& priority_queue) : env_(env), priority_queue_(priority_queue) {};

  void solve() {
    uint32_t start_node = env_.get_start_node();
    const auto& start_node_data = env_.get_pool()[start_node];
    priority_queue_.push(start_node, start_node_data.g + start_node_data.h);

    std::vector<uint32_t> neighbors;

    while (!priority_queue_.empty()) {
      uint32_t current_node = priority_queue_.pop();

      if (env_.is_goal(env_.get_state(current_node))) {
        return;
      }

      neighbors.clear();
      env_.get_successors(current_node, neighbors);

      for (uint32_t neighbor : neighbors) {
        const auto& neighbor_data = env_.get_pool()[neighbor];
        uint32_t priority = neighbor_data.g + neighbor_data.h;
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