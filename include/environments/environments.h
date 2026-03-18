// include/environments/environments.h

#pragma once

#include <algorithm>
#include <array>
#include <iostream>
#include <fstream>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <map>
#include <utility>

#include "node.h"
#include "phmap.h"

class GridEnvironment { 

public:

  GridEnvironment(uint32_t w, uint32_t h, uint32_t seed = 41);

  void reset_search() { pool_.prepare_for_search(); }

  void get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors);
  inline uint32_t get_heuristic(uint32_t id) const { 
    return (w_ - 1 - (id % w_)) + (h_ - 1 - (id / w_)); // Manhattan Distance
  }
  bool is_goal(uint32_t id) const { return id == goal_id_; }

  uint32_t get_start_node();

  inline uint32_t get_edge_cost(uint32_t u, uint32_t v) const { return grid_map_[v]; }

  NodePool& get_pool() { return pool_; }
  const NodePool& get_pool() const { return pool_; }

private:

  uint32_t w_;
  uint32_t h_;
  uint32_t goal_id_;
  std::vector<uint8_t> grid_map_; // state_id -> cost
  NodePool pool_;

};

class PancakeEnvironment {

public:

  static constexpr size_t N = 48;
  using T = std::array<uint8_t, N>;

  // FNV-1a Hash
  struct StateHash {
    uint32_t operator()(const T& s) const {
      uint32_t hash = 2166136261u;
      for (uint8_t val : s) {
        hash ^= val;
        hash *= 16777619u;
      }
      return hash;
    }
  };

  PancakeEnvironment(uint32_t seed = 42, uint32_t capacity = 1e7);

  void reset_search() { pool_.prepare_for_search(); }

  void get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors);


  inline uint32_t get_heuristic(uint32_t id) const {
    const T& state = node_states_[id];
    uint32_t gaps = 0;
    
    for (size_t i = 0; i < state.size() - 1; i++) {
      if (std::abs((int)state[i] - (int)state[i+1]) > 1) {
        gaps++;
      }
    }
    
    // Plate boundary check: goal is 0, 1, ..., N-1. Plate is effectively N.
    if (state[N-1] != (uint8_t)(N - 1)) {
      gaps++;
    }

    return gaps;
  }

  bool is_goal(uint32_t id) const { return id == goal_id_; };

  uint32_t get_start_node();

  inline uint32_t get_edge_cost(uint32_t u, uint32_t v) const { return 1; }

  NodePool& get_pool() { return pool_; }
  const NodePool& get_pool() const { return pool_; }

  uint32_t generate_start_node();

  const T& get_state(uint32_t id) const { return node_states_[id]; }

  uint32_t get_or_create_id(const T& state);

private:

  uint32_t capacity_;
  NodePool pool_;

  std::vector<T> node_states_; // node_handle -> state_id
  phmap::flat_hash_map<T, uint32_t, StateHash> lookup_table_;

  T start_state_;
  T goal_state_;
  uint32_t goal_id_;
  uint32_t seed_;

};

class SlidingTileEnvironment {

public:

  using T = uint64_t;

  SlidingTileEnvironment(int instance_index, std::string filename, uint32_t capacity = 1000000);

  void reset_search() { pool_.prepare_for_search(); }

  void get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors);

  inline uint32_t get_heuristic(uint32_t id) const { // Manhattan Distance
    T state = node_states_[id];
    uint32_t h = 0;

    for (int i = 0; i < 16; i++) {
      int tile = get_tile(state, i);
      if (tile == 0) continue;

      h += MD_TABLE[tile][i];
    }
    return h;
  }



  inline bool is_goal(uint32_t id) const { return id == goal_id_; };

  inline uint32_t get_edge_cost(uint32_t u, uint32_t v) const { return 1; }

  NodePool& get_pool() {return pool_; }
  const NodePool& get_pool() const { return pool_; }

  uint32_t get_start_node();

  inline uint8_t get_tile(T state, int index) const { return (state >> ((15 - index) * 4)) & 0xF; };

  T get_state(uint32_t id) const { return node_states_[id]; }
  uint32_t get_or_create_id(T state);

private:

  uint32_t capacity_;
  NodePool pool_;

  std::vector<T> node_states_; 
  phmap::flat_hash_map<T, uint32_t> lookup_table_;

  T start_state_;
  T goal_state_;
  uint32_t goal_id_;

  static const int8_t MOVES[16][4];
  static const uint8_t MD_TABLE[16][16];

};

class RandomGridEnvironment {

public:

  RandomGridEnvironment(uint32_t w, uint32_t h, uint32_t max_edge_cost, uint32_t seed = 42);

  void reset_search() { pool_.prepare_for_search(); }

  void get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors);

  inline uint32_t get_heuristic(uint32_t id) const { 
    return (w_ - 1 - (id % w_)) + (h_ - 1 - (id / w_)); 
  }

  bool is_goal(uint32_t id) const { return id == goal_id_; }

  uint32_t get_start_node() { return 0; };

  inline uint32_t get_edge_cost(uint32_t u, uint32_t v) const { return grid_map_[v]; }

  NodePool& get_pool() { return pool_; }
  const NodePool& get_pool() const { return pool_; }

private:

  uint32_t w_;
  uint32_t h_;
  uint32_t goal_id_;
  std::vector<uint8_t> grid_map_; // Stores cost (0 = obstacle, 1-255 = cost)
  NodePool pool_;

};

class HeavyPancakeEnvironment {

public:

  static constexpr size_t N = 10;
  using T = std::array<uint8_t, N>;

  // FNV-1a Hash
  struct StateHash {
    uint32_t operator()(const T& s) const {
      uint32_t hash = 2166136261u;
      for (uint8_t val : s) {
        hash ^= val;
        hash *= 16777619u;
      }
      return hash;
    }
  };

  HeavyPancakeEnvironment(uint32_t seed = 42, uint32_t capacity = 1e7);

  void reset_search() { pool_.prepare_for_search(); }

  void get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors);

  inline uint32_t get_heuristic(uint32_t id) const {
    const T& state = node_states_[id];
    uint32_t h = 0;
    
    for (size_t i = 0; i < state.size() - 1; i++) {
      if (std::abs((int)state[i] - (int)state[i+1]) > 1) {
        if (use_heavy_heuristic_) {
          h += std::min(state[i], state[i+1]);
        } else {
          h += 1;
        }
      }
    }

    // Plate boundary check
    if (state[N-1] != (uint8_t)(N - 1)) {
        if (use_heavy_heuristic_) {
          h += std::min((int)state[N-1], (int)N);
        } else {
          h += 1;
        }
    }
    
    return h;
  }

  bool is_goal(uint32_t id) const { return id == goal_id_; };

  uint32_t get_start_node();

  // Heavy Cost: max(first_pancake, last_pancake_in_flip_prefix)
  inline uint32_t get_edge_cost(uint32_t u, uint32_t v) const { 
    const T& s_u = node_states_[u];
    const T& s_v = node_states_[v];
    
    // Find the rightmost difference to determine the flip index k
    for (int k = N - 1; k >= 0; --k) {
        if (s_u[k] != s_v[k]) {
            // Flip was length k+1 (indices 0 to k)
            // Cost is max(s_u[0], s_u[k])
            return std::max(s_u[0], s_u[k]);
        }
    }
    return 1; // Should not happen if u != v
  }

  NodePool& get_pool() { return pool_; }
  const NodePool& get_pool() const { return pool_; }

  uint32_t generate_start_node();
  const T& get_state(uint32_t id) const { return node_states_[id]; }
  uint32_t get_or_create_id(const T& state);

  void set_heavy_heuristic(bool use_heavy) { use_heavy_heuristic_ = use_heavy; }

private:

  uint32_t capacity_;
  NodePool pool_;

  std::vector<T> node_states_; 
  phmap::flat_hash_map<T, uint32_t, StateHash> lookup_table_;

  T start_state_;
  T goal_state_;
  uint32_t goal_id_;
  uint32_t seed_;
  bool use_heavy_heuristic_ = true;

};

class HeavySlidingTileEnvironment {

public:

  using T = uint64_t;

  HeavySlidingTileEnvironment(int instance_index, std::string filename, uint32_t capacity = 1000000);

  void reset_search() { pool_.prepare_for_search(); }

  void get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors);

  inline uint32_t get_heuristic(uint32_t id) const { // Manhattan Distance
    T state = node_states_[id];
    uint32_t h = 0;
    for (int i = 0; i < 16; i++) {
      int tile = get_tile(state, i);
      if (tile == 0) continue;
      if (use_heavy_heuristic_) {
        h += MD_TABLE[tile][i] * tile; // Heavy Heuristic: MD * weight
      } else {
        h += MD_TABLE[tile][i]; // Standard Heuristic: unit weight
      }
    }
    return h;
  }

  inline bool is_goal(uint32_t id) const { return id == goal_id_; };

  // Heavy Cost: Cost is the value of the tile moved.
  inline uint32_t get_edge_cost(uint32_t u, uint32_t v) const { 
    T s_u = node_states_[u];
    T s_v = node_states_[v];
    
    // XOR exposes the two changed positions (the tile X and the blank 0)
    // 0 ^ X = X. So the diff will have the hex digit X in two places.
    T diff = s_u ^ s_v;
    
    // Find first non-zero nibble
    if (diff == 0) return 0;
    int trailing_zeros = __builtin_ctzll(diff);
    int nibble_shift = (trailing_zeros / 4) * 4;
    
    // Extract the tile value
    return (diff >> nibble_shift) & 0xF;
  }

  NodePool& get_pool() {return pool_; }
  const NodePool& get_pool() const { return pool_; }

  uint32_t get_start_node();
  inline uint8_t get_tile(T state, int index) const { return (state >> ((15 - index) * 4)) & 0xF; };
  T get_state(uint32_t id) const { return node_states_[id]; }
  uint32_t get_or_create_id(T state);

  void set_heavy_heuristic(bool use_heavy) { use_heavy_heuristic_ = use_heavy; }

private:

  uint32_t capacity_;
  NodePool pool_;

  std::vector<T> node_states_; 
  phmap::flat_hash_map<T, uint32_t> lookup_table_;

  T start_state_;
  T goal_state_;
  uint32_t goal_id_;
  bool use_heavy_heuristic_ = true;

  // Re-declare needed tables 
  static const int8_t MOVES[16][4];
  static const uint8_t MD_TABLE[16][16];

};

class MSAEnvironment {

public:
  static constexpr size_t N = 5;
  using T = std::array<uint16_t, N>; // State is the current index in each sequence

  struct StateHash {
    uint32_t operator()(const T& s) const {
      uint32_t hash = 2166136261u;
      for (uint16_t val : s) {
        hash ^= val;
        hash *= 16777619u;
      }
      return hash;
    }
  };

  MSAEnvironment(const std::vector<std::string>& sequences, uint32_t capacity = 10000000);

  void reset_search() { pool_.prepare_for_search(); }

  void get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors);

  uint32_t get_heuristic(uint32_t id) const;

  bool is_goal(uint32_t id) const;

  uint32_t get_start_node();

  uint32_t get_edge_cost(uint32_t u, uint32_t v) const;

  NodePool& get_pool() { return pool_; }
  const NodePool& get_pool() const { return pool_; }

  uint32_t get_or_create_id(const T& state);
  const T& get_state(uint32_t id) const { return node_states_[id]; }

private:
  void compute_pairwise_heuristics();
  uint32_t compute_transition_cost(const T& u_state, const T& v_state) const;
  
  static int aa_to_idx(char c);

  std::vector<std::vector<uint8_t>> encoded_sequences_;
  int cost_table_[21][21];

  std::vector<int> seq_lengths_;
  std::vector<std::vector<std::vector<uint32_t>>> h_tables_;

  uint32_t capacity_;
  NodePool pool_;

  std::vector<T> node_states_; 
  phmap::flat_hash_map<T, uint32_t, StateHash> lookup_table_;

  T start_state_;
  T goal_state_;
  uint32_t goal_id_;

  static constexpr uint32_t GAP_COST = 1;
  static constexpr uint32_t MISMATCH_COST = 2;
};
