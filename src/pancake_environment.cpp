// src/pancake_environment.cpp

#include "environments/environments.h"

#include <algorithm>
#include <numeric>
#include <random>

PancakeEnvironment::PancakeEnvironment(uint32_t seed, uint32_t capacity) : capacity_(capacity), seed_(seed) {

  std::iota(goal_state_.begin(), goal_state_.end(), 0);

  pool_.reserve(capacity_);
  node_states_.reserve(capacity_);
  h_costs_.reserve(capacity_);
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
  h_costs_.push_back(INF_COST);
  lookup_table_.insert({state, new_id});

  return new_id;
}

uint32_t PancakeEnvironment::get_start_node() {
  return get_or_create_id(start_state_);
}

void PancakeEnvironment::get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors) {
  neighbors.clear();

  const T current_state = node_states_[u_id];
  uint32_t parent_h = get_heuristic(u_id);

  static thread_local T temp_state;

  for (size_t k = 2; k <= N; k++) {
    // A flip of length k changes exactly one gap: the boundary between positions k-1 and k.
    // After the flip, the element at k-1 becomes the old element at 0.
    uint32_t old_gap, new_gap;
    if (k < N) {
      old_gap = (std::abs((int)current_state[k-1] - (int)current_state[k]) > 1) ? 1u : 0u;
      new_gap = (std::abs((int)current_state[0]   - (int)current_state[k]) > 1) ? 1u : 0u;
    } else {
      // k == N: boundary is with the implicit plate (goal value N-1 at position N-1).
      old_gap = (current_state[N-1] != (uint8_t)(N-1)) ? 1u : 0u;
      new_gap = (current_state[0]   != (uint8_t)(N-1)) ? 1u : 0u;
    }
    uint32_t new_h = parent_h + new_gap - old_gap;

    temp_state = current_state;
    std::reverse(temp_state.begin(), temp_state.begin() + k);
    uint32_t v_id = get_or_create_id(temp_state);
    if (h_costs_[v_id] == INF_COST) h_costs_[v_id] = new_h;
    neighbors.push_back(v_id);
  }
}

HeavyPancakeEnvironment::HeavyPancakeEnvironment(uint32_t seed, uint32_t capacity) : capacity_(capacity), seed_(seed) {
  std::iota(goal_state_.begin(), goal_state_.end(), 0);
  pool_.reserve(capacity_);
  node_states_.reserve(capacity_);
  h_costs_.reserve(capacity_);
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
  h_costs_.push_back(INF_COST);
  lookup_table_.insert({state, new_id});
  return new_id;
}

uint32_t HeavyPancakeEnvironment::get_start_node() {
  return get_or_create_id(start_state_);
}

void HeavyPancakeEnvironment::get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors) {
  neighbors.clear();
  const T current_state = node_states_[u_id];
  uint32_t parent_h = get_heuristic(u_id);
  static thread_local T temp_state;

  for (size_t k = 2; k <= N; k++) {
    // A flip of length k changes exactly one gap: the boundary between positions k-1 and k.
    // After the flip, the element at k-1 becomes the old element at 0.
    auto gap_cost = [&](uint8_t a, uint8_t b) -> uint32_t {
      if (std::abs((int)a - (int)b) <= 1) return 0u;
      return use_heavy_heuristic_ ? (uint32_t)std::min(a, b) : 1u;
    };

    uint32_t old_gap, new_gap;
    if (k < N) {
      old_gap = gap_cost(current_state[k-1], current_state[k]);
      new_gap = gap_cost(current_state[0],   current_state[k]);
    } else {
      // k == N: boundary is with the implicit plate (goal value N-1 at position N-1).
      auto plate_gap = [&](uint8_t val) -> uint32_t {
        if (val == (uint8_t)(N-1)) return 0u;
        return use_heavy_heuristic_ ? (uint32_t)std::min((int)val, (int)N) : 1u;
      };
      old_gap = plate_gap(current_state[N-1]);
      new_gap = plate_gap(current_state[0]);
    }
    uint32_t new_h = parent_h + new_gap - old_gap;

    temp_state = current_state;
    std::reverse(temp_state.begin(), temp_state.begin() + k);
    uint32_t v_id = get_or_create_id(temp_state);
    if (h_costs_[v_id] == INF_COST) h_costs_[v_id] = new_h;
    neighbors.push_back(v_id);
  }
}