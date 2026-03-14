// src/pancake_environment.cpp

#include "environments/environments.h"

#include <algorithm>
#include <numeric>
#include <random>

PancakeEnvironment::PancakeEnvironment(uint32_t seed, uint32_t capacity) : capacity_(capacity), seed_(seed) {

  std::iota(goal_state_.begin(), goal_state_.end(), 0);

  pool_.reserve(capacity_);
  node_states_.reserve(capacity_);
  lookup_table_.reserve(capacity_);

  goal_id_ = get_or_create_id(goal_state_);
}

uint32_t PancakeEnvironment::generate_start_node() {
  start_state_ = goal_state_;

  std::mt19937 g(seed_);
  std::shuffle(start_state_.begin(), start_state_.end(), g);

  return get_or_create_id(start_state_);
}

uint32_t PancakeEnvironment::get_or_create_id(const T& state) {
  auto it = lookup_table_.find(state);
  if (it != lookup_table_.end()) {
    return it->second;
  }

  uint32_t new_id = pool_.create_new_state_id();
  node_states_.push_back(state);
  lookup_table_.insert({state, new_id});

  return new_id;
}

uint32_t PancakeEnvironment::get_start_node() {
  return get_or_create_id(start_state_);
}


void PancakeEnvironment::get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors) {
  neighbors.clear();

  const T current_state = node_states_[u_id];

  static thread_local T temp_state;

  for (size_t k = 2; k <= current_state.size(); k++) {
    temp_state = current_state;

    std::reverse(temp_state.begin(), temp_state.begin() + k);
    uint32_t v_id = get_or_create_id(temp_state);
    neighbors.push_back(v_id);
  }
}

HeavyPancakeEnvironment::HeavyPancakeEnvironment(uint32_t seed, uint32_t capacity) : capacity_(capacity), seed_(seed) {
  std::iota(goal_state_.begin(), goal_state_.end(), 0);
  pool_.reserve(capacity_);
  node_states_.reserve(capacity_);
  lookup_table_.reserve(capacity_);
  goal_id_ = get_or_create_id(goal_state_);
}

uint32_t HeavyPancakeEnvironment::generate_start_node() {
  start_state_ = goal_state_;
  std::mt19937 g(seed_); 
  std::shuffle(start_state_.begin(), start_state_.end(), g);
  return get_or_create_id(start_state_);
}

uint32_t HeavyPancakeEnvironment::get_or_create_id(const T& state) {
  auto it = lookup_table_.find(state);
  if (it != lookup_table_.end()) {
    return it->second;
  }
  uint32_t new_id = pool_.create_new_state_id();
  node_states_.push_back(state);
  lookup_table_.insert({state, new_id});
  return new_id;
}

uint32_t HeavyPancakeEnvironment::get_start_node() {
  return get_or_create_id(start_state_);
}

void HeavyPancakeEnvironment::get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors) {
  neighbors.clear();
  const T current_state = node_states_[u_id];
  static thread_local T temp_state;

  for (size_t k = 2; k <= current_state.size(); k++) {
    temp_state = current_state;
    std::reverse(temp_state.begin(), temp_state.begin() + k);
    uint32_t v_id = get_or_create_id(temp_state);
    neighbors.push_back(v_id);
  }
}