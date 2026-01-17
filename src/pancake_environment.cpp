// src/pancake_environment.cpp

#include "environments/environments.h"
#include <algorithm>
#include <random>

PancakeEnvironment::PancakeEnvironment(uint32_t seed, uint32_t capacity) : capacity_(capacity) {
  for (int i = 0; i < 48; i++) {
    goal_state_[i] = i;
  }

  start_state_ = goal_state_;
  std::mt19937 g(seed);
  std::shuffle(start_state_.begin(), start_state_.end(), g);

  pool_.reserve(capacity_);
  node_states_.reserve(capacity_);
  lookup_table_.reserve(capacity_);
}

uint32_t PancakeEnvironment::get_start_node() {
  uint32_t h_val = get_heuristic(start_state_);
  uint32_t handle = pool_.allocate(NODE_NULL, 0, h_val);

  node_states_.push_back(start_state_);
  lookup_table_[start_state_] = handle;

  return handle;
}

uint32_t PancakeEnvironment::get_heuristic(const T& state) const {
  uint32_t gaps = 0;
  
  for (int i = 0; i < 47; i++) {
    int diff = std::abs((int)state[i] - (int)state[i+1]);
    if (diff > 1) {
      gaps++;
    }
  }
  return gaps;
}

void PancakeEnvironment::get_successors(uint32_t parent_handle, std::vector<uint32_t>& neighbors) {
  const uint32_t parent_g = pool_[parent_handle].g;
  const T& parent_state = node_states_[parent_handle];

  static thread_local T temp_state;

  for (int i = 2; i <= 48; i++) {
    temp_state = parent_state;

    std::reverse(temp_state.begin(), temp_state.begin() + i);
    uint32_t new_g = parent_g + 1;
    auto it = lookup_table_.find(temp_state);

    if (it == lookup_table_.end()) {
      // NEW NODE
      uint32_t h_val = get_heuristic(temp_state);
      uint32_t n_handle = pool_.allocate(parent_handle, new_g, h_val);

      node_states_.push_back(temp_state);
      lookup_table_.insert({temp_state, n_handle});

      neighbors.push_back(n_handle);
    }
    else {
      // EXISTING NODE
      uint32_t n_handle = it->second;
      if (new_g < pool_[n_handle].g) {
        pool_[n_handle].g = new_g;
        pool_[n_handle].parent = parent_handle;
        neighbors.push_back(n_handle);
      }
    }
  }
}