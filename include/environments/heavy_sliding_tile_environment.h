// include/environments/heavy_sliding_tile_environment.h

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include "node.h"
#include "phmap.h"

// 15-puzzle with heavy (non-unit) edge costs.
// Edge cost of moving a tile is the tile's value.
// Heuristic is sum of Manhattan distances weighted by tile value (MD * tile),
// which is admissible since the tile must move at least MD times and costs >= tile value.
// Can fall back to standard (unit-weight) MD heuristic via set_heavy_heuristic(false).
class HeavySlidingTileEnvironment {

public:

  using T = uint64_t;

  HeavySlidingTileEnvironment(int instance_index, std::string filename, uint32_t capacity = 1000000);

  void reset_search() { pool_.prepare_for_search(); }

  void get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors);

  inline uint32_t get_heuristic(uint32_t id) const {
    if (h_costs_[id] != INF_COST) return h_costs_[id];
    T state = node_states_[id];
    uint32_t h = 0;
    for (int i = 0; i < 16; i++) {
      int tile = get_tile(state, i);
      if (tile == 0) continue;
      h += use_heavy_heuristic_ ? MD_TABLE[tile][i] * tile : MD_TABLE[tile][i];
    }
    h_costs_[id] = h;
    return h;
  }

  inline bool is_goal(uint32_t id) const { return id == goal_id_; }

  // Cost is the value of the tile being moved.
  // XOR exposes the two changed nibbles; the non-zero nibble value is the tile.
  inline uint32_t get_edge_cost(uint32_t u, uint32_t v) const {
    T diff = node_states_[u] ^ node_states_[v];
    if (diff == 0) return 0;
    int nibble_shift = (__builtin_ctzll(diff) / 4) * 4;
    return (diff >> nibble_shift) & 0xF;
  }

  NodePool& get_pool() { return pool_; }
  const NodePool& get_pool() const { return pool_; }

  uint32_t get_start_node();
  inline uint8_t get_tile(T state, int index) const { return (state >> ((15 - index) * 4)) & 0xF; }
  T get_state(uint32_t id) const { return node_states_[id]; }
  uint32_t get_or_create_id(T state);
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
  std::vector<uint8_t> blank_pos_;          // cached blank tile position for each node id
  mutable std::vector<uint32_t> h_costs_;   // cached heuristic values; INF_COST = not yet computed
  phmap::flat_hash_map<T, uint32_t> lookup_table_;

  T start_state_;
  T goal_state_;
  uint32_t goal_id_;
  bool use_heavy_heuristic_ = true;

  static const int8_t MOVES[16][4];
  static const uint8_t MD_TABLE[16][16];

};
