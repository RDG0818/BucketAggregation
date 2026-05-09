// include/environments/sliding_tile_environment.h

#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include "node.h"
#include "phmap.h"

// 15-puzzle (4x4 sliding tile) with unit edge costs.
// State is packed into a uint64_t as 16 nibbles (4 bits per tile, MSB = position 0).
// Heuristic is the sum of Manhattan distances of each tile to its goal position,
// precomputed in MD_TABLE[tile][position].
class SlidingTileEnvironment {

public:

  using T = uint64_t;

  SlidingTileEnvironment(int instance_index, std::string filename, uint32_t capacity = 1000000);

  void reset_search() { pool_.prepare_for_search(); }

  void get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors);

  inline uint32_t get_heuristic(uint32_t id) const {
    if (h_costs_[id] != INF_COST) return h_costs_[id];
    T state = node_states_[id];
    uint32_t h = 0;
    for (int i = 0; i < 16; i++) {
      int tile = get_tile(state, i);
      if (tile == 0) continue;
      h += MD_TABLE[tile][i];
    }
    h_costs_[id] = h;
    return h;
  }

  inline bool is_goal(uint32_t id) const { return id == goal_id_; }

  inline uint32_t get_edge_cost(uint32_t u, uint32_t v) const { return 1; }

  NodePool& get_pool() { return pool_; }
  const NodePool& get_pool() const { return pool_; }

  uint32_t get_start_node();
  inline uint8_t get_tile(T state, int index) const { return (state >> ((15 - index) * 4)) & 0xF; }
  T get_state(uint32_t id) const { return node_states_[id]; }
  uint32_t get_or_create_id(T state);

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

  static const int8_t MOVES[16][4];
  static const uint8_t MD_TABLE[16][16];

};
