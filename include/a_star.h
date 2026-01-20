#pragma once

#include <vector>
#include <cstdint>

template <typename E, typename PQ> 
class AStar {

public:

  AStar(E& env, PQ& priority_queue) : env_(env), priority_queue_(priority_queue) {};

  void solve() {
    uint32_t start_node = env_.get_start_node();
    priority_queue_.push(start_node);

    std::vector<uint32_t> neighbors;

    while (!priority_queue_.empty()) {
      uint32_t current_node = priority_queue_.pop();

      if (env_.is_goal(env_.get_state(current_node))) {
        return;
      }

      neighbors.clear();
      env_.get_successors(current_node, neighbors);

      for (uint32_t neighbor : neighbors) {
        if (priority_queue_.contains(neighbor)) {
          priority_queue_.decrease_key(neighbor);
        } else {
          priority_queue_.push(neighbor);
        }
      }
    }
  }

private:

  E& env_;
  PQ& priority_queue_;

};