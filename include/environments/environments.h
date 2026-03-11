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

  static constexpr size_t N = 40;
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
      int diff = std::abs((int)state[i] - (int)state[i+1]);
      if (diff > 1) {
        gaps++;
      }
    }
    if (gaps == 0 && !is_goal(id)) {
        return 1;
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

  static constexpr size_t N = 11;
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
      int diff = std::abs((int)state[i] - (int)state[i+1]);
      if (diff > 1) {
        h += std::min(state[i], state[i+1]);
      }
    }

    if (h == 0 && !is_goal(id)) {
        // This means we have a reversed sequence, e.g., [4, 3, 2, 1]
        // A single flip of the whole stack is needed.
        return std::max(state[0], state.back());
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

private:

  uint32_t capacity_;
  NodePool pool_;

  std::vector<T> node_states_; 
  phmap::flat_hash_map<T, uint32_t, StateHash> lookup_table_;

  T start_state_;
  T goal_state_;
  uint32_t goal_id_;

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
      h += MD_TABLE[tile][i] * tile; // Heavy Heuristic: MD * weight
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

private:

  uint32_t capacity_;
  NodePool pool_;

  std::vector<T> node_states_; 
  phmap::flat_hash_map<T, uint32_t> lookup_table_;

  T start_state_;
  T goal_state_;
  uint32_t goal_id_;

  // Re-declare needed tables 
  static const int8_t MOVES[16][4];
  static const uint8_t MD_TABLE[16][16];

};

class MSAEnvironment {
private:
    // Private inner class for PAM250 scoring
    struct PAM250 {
        std::map<std::pair<char, char>, int> matrix;

        void add(char a, char b, int score) {
            if (a > b) std::swap(a, b);
            matrix[{a, b}] = score;
        }

        PAM250() {
            add('A', 'A',  2); add('A', 'R', -2); add('R', 'R',  6); add('A', 'N',  0); add('R', 'N',  0); add('N', 'N',  2);
            add('A', 'D',  0); add('R', 'D', -1); add('N', 'D',  2); add('D', 'D',  4); add('A', 'C', -2); add('R', 'C', -4);
            add('N', 'C', -4); add('D', 'C', -5); add('C', 'C', 12); add('A', 'Q',  0); add('R', 'Q',  1); add('N', 'Q',  1);
            add('D', 'Q',  2); add('C', 'Q', -5); add('Q', 'Q',  4); add('A', 'E',  0); add('R', 'E', -1); add('N', 'E',  1);
            add('D', 'E',  3); add('C', 'E', -5); add('Q', 'E',  2); add('E', 'E',  4); add('A', 'G',  1); add('R', 'G', -3);
            add('N', 'G',  0); add('D', 'G',  1); add('C', 'G', -3); add('Q', 'G', -1); add('E', 'G',  0); add('G', 'G',  5);
            add('A', 'H', -1); add('R', 'H',  2); add('N', 'H',  2); add('D', 'H',  1); add('C', 'H', -3); add('Q', 'H',  3);
            add('E', 'H',  1); add('G', 'H', -2); add('H', 'H',  6); add('A', 'I', -1); add('R', 'I', -2); add('N', 'I', -2);
            add('D', 'I', -2); add('C', 'I', -2); add('Q', 'I', -2); add('E', 'I', -2); add('G', 'I', -3); add('H', 'I', -2);
            add('I', 'I',  5); add('A', 'L', -2); add('R', 'L', -3); add('N', 'L', -3); add('D', 'L', -4); add('C', 'L', -6);
            add('Q', 'L', -2); add('E', 'L', -3); add('G', 'L', -4); add('H', 'L', -2); add('I', 'L',  2); add('L', 'L',  6);
            add('A', 'K', -1); add('R', 'K',  3); add('N', 'K',  1); add('D', 'K',  0); add('C', 'K', -5); add('Q', 'K',  1);
            add('E', 'K',  0); add('G', 'K', -2); add('H', 'K',  0); add('I', 'K', -2); add('L', 'K', -3); add('K', 'K',  5);
            add('A', 'M', -1); add('R', 'M',  0); add('N', 'M', -2); add('D', 'M', -3); add('C', 'M', -5); add('Q', 'M', -1);
            add('E', 'M', -2); add('G', 'M', -3); add('H', 'M', -2); add('I', 'M',  2); add('L', 'M',  4); add('K', 'M',  0);
            add('M', 'M',  6); add('A', 'F', -4); add('R', 'F', -4); add('N', 'F', -4); add('D', 'F', -6); add('C', 'F', -4);
            add('Q', 'F', -5); add('E', 'F', -5); add('G', 'F', -5); add('H', 'F', -2); add('I', 'F',  1); add('L', 'F',  2);
            add('K', 'F', -5); add('M', 'F',  0); add('F', 'F',  9); add('A', 'P',  1); add('R', 'P',  0); add('N', 'P', -1);
            add('D', 'P', -1); add('C', 'P', -3); add('Q', 'P',  0); add('E', 'P', -1); add('G', 'P', -1); add('H', 'P',  0);
            add('I', 'P', -2); add('L', 'P', -3); add('K', 'P', -1); add('M', 'P', -2); add('F', 'P', -5); add('P', 'P',  6);
            add('A', 'S',  1); add('R', 'S',  0); add('N', 'S',  1); add('D', 'S',  0); add('C', 'S',  0); add('Q', 'S', -1);
            add('E', 'S',  0); add('G', 'S',  1); add('H', 'S', -1); add('I', 'S', -1); add('L', 'S', -3); add('K', 'S',  0);
            add('M', 'S', -2); add('F', 'S', -3); add('P', 'S',  1); add('S', 'S',  3); add('A', 'T',  1); add('R', 'T', -1);
            add('N', 'T',  0); add('D', 'T',  0); add('C', 'T', -2); add('Q', 'T', -1); add('E', 'T',  0); add('G', 'T',  0);
            add('H', 'T', -1); add('I', 'T',  0); add('L', 'T', -2); add('K', 'T',  0); add('M', 'T', -1); add('F', 'T', -2);
            add('P', 'T',  0); add('S', 'T',  1); add('T', 'T',  3); add('A', 'W', -6); add('R', 'W',  2); add('N', 'W', -4);
            add('D', 'W', -7); add('C', 'W', -8); add('Q', 'W', -5); add('E', 'W', -7); add('G', 'W', -7); add('H', 'W', -3);
            add('I', 'W', -5); add('L', 'W', -2); add('K', 'W', -3); add('M', 'W', -4); add('F', 'W',  0); add('P', 'W', -6);
            add('S', 'W', -2); add('T', 'W', -5); add('W', 'W', 17); add('A', 'Y', -3); add('R', 'Y', -4); add('N', 'Y', -2);
            add('D', 'Y', -4); add('C', 'Y',  0); add('Q', 'Y', -4); add('E', 'Y', -4); add('G', 'Y', -5); add('H', 'Y',  0);
            add('I', 'Y', -1); add('L', 'Y', -1); add('K', 'Y', -4); add('M', 'Y', -2); add('F', 'Y',  7); add('P', 'Y', -5);
            add('S', 'Y', -3); add('T', 'Y', -3); add('W', 'Y',  0); add('Y', 'Y', 10); add('A', 'V',  0); add('R', 'V', -2);
            add('N', 'V', -2); add('D', 'V', -2); add('C', 'V', -2); add('Q', 'V', -2); add('E', 'V', -2); add('G', 'V', -1);
            add('H', 'V', -2); add('I', 'V',  4); add('L', 'V',  2); add('K', 'V', -2); add('M', 'V',  2); add('F', 'V', -1);
            add('P', 'V', -1); add('S', 'V', -1); add('T', 'V',  0); add('W', 'V', -6); add('Y', 'V', -2); add('V', 'V',  4);
        }

        // The papers use reversed scores for their cost-minimization search.
        // score(a,b) = -PAM(a,b)
        int getCost(char a, char b) const {
            if (a > b) std::swap(a, b);
            auto it = matrix.find({a, b});
            if (it != matrix.end()) {
                return -it->second;
            }
            return 4; // Default penalty for unknown characters
        }
    };

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
  uint32_t get_pairwise_cost(char a, char b) const;

  std::vector<std::string> sequences_;
  std::vector<int> seq_lengths_;
  std::vector<std::vector<std::vector<uint32_t>>> h_tables_;

  uint32_t capacity_;
  NodePool pool_;

  std::vector<T> node_states_; 
  phmap::flat_hash_map<T, uint32_t, StateHash> lookup_table_;

  T start_state_;
  T goal_state_;
  uint32_t goal_id_;

  PAM250 pam250_scorer_;
  static constexpr uint32_t GAP_COST = 8;
};
