// include/environments/heavy_pancake_environment.h

#pragma once

#include <algorithm>  // std::min, std::max
#include <array>
#include <cstdint>
#include <cstdlib>    // std::abs
#include <vector>
#include "node.h"
#include "fnv_hash.h"
#include "phmap.h"

// N-pancake sorting problem with heavy (non-unit) edge costs.
// Edge cost of a flip of length k is max(stack[0], stack[k-1]).
// Heuristic is a weighted gap heuristic: each gap contributes min(stack[i], stack[i+1])
// instead of 1, making it admissible for heavy costs.
// Can fall back to the standard (unit) gap heuristic via set_heavy_heuristic(false).
class HeavyPancakeEnvironment {

public:

  static constexpr size_t N = 10;
  using T = std::array<uint8_t, N>;

  HeavyPancakeEnvironment(uint32_t seed = 42, uint32_t capacity = 1e7);

  void reset_search() { pool_.prepare_for_search(); }

  void get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors);

  inline uint32_t get_heuristic(uint32_t id) const {
    if (h_costs_[id] != INF_COST) return h_costs_[id];
    const T& state = node_states_[id];
    uint32_t h = 0;
    for (size_t i = 0; i < state.size() - 1; i++) {
      if (std::abs((int)state[i] - (int)state[i+1]) > 1) {
        h += use_heavy_heuristic_ ? std::min(state[i], state[i+1]) : 1u;
      }
    }
    // Plate boundary check
    if (state[N-1] != (uint8_t)(N - 1)) {
      h += use_heavy_heuristic_ ? std::min((int)state[N-1], (int)N) : 1;
    }
    h_costs_[id] = h;
    return h;
  }

  bool is_goal(uint32_t id) const { return id == goal_id_; }

  uint32_t get_start_node();

  // Cost is max(first_pancake, last_pancake_in_flip_prefix).
  inline uint32_t get_edge_cost(uint32_t u, uint32_t v) const {
    const T& s_u = node_states_[u];
    const T& s_v = node_states_[v];
    for (int k = N - 1; k >= 0; --k) {
      if (s_u[k] != s_v[k]) return std::max(s_u[0], s_u[k]);
    }
    return 1; // unreachable if u != v
  }

  NodePool& get_pool() { return pool_; }
  const NodePool& get_pool() const { return pool_; }

  uint32_t generate_start_node();
  const T& get_state(uint32_t id) const { return node_states_[id]; }
  uint32_t get_or_create_id(const T& state);
  void set_heavy_heuristic(bool use_heavy) {
    if (use_heavy_heuristic_ != use_heavy) {
      use_heavy_heuristic_ = use_heavy;
      std::fill(h_costs_.begin(), h_costs_.end(), INF_COST); // cached h values are now stale
    }
  }

private:

  uint32_t capacity_;
  NodePool pool_;

  std::vector<T> node_states_;
  mutable std::vector<uint32_t> h_costs_;   // cached heuristic values; INF_COST = not yet computed
  phmap::flat_hash_map<T, uint32_t, FnvHash> lookup_table_;

  T start_state_;
  T goal_state_;
  uint32_t goal_id_;
  uint32_t seed_;
  bool use_heavy_heuristic_ = true;

};
