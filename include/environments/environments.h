// include/environments/environments.h

#pragma once

#include <array>
#include <random>
#include <unordered_map>

#include "node.h"

class GridEnvironment { 

public:

  GridEnvironment(uint32_t w, uint32_t h, uint32_t seed = 42);

  void get_successors(uint32_t node_index, std::vector<uint32_t>& neighbors);
  uint32_t get_heuristic(uint32_t state_id) const { 
    return (w_ - 1 - (state_id % w_)) + (h_ - 1 - (state_id / w_)); // Manhattan Distance
  }
  bool is_goal(uint32_t state_id) const { return state_id == goal_id_; }

  uint32_t get_start_node() { // 0 indicates start state
    uint32_t handle = pool_.allocate(NODE_NULL, 0, get_heuristic(0));

    node_states_.push_back(0);
    lookup_table_[0] = handle;
    return handle;
  }

  uint8_t get_cell_cost(uint32_t state_id) const { return grid_map_[state_id]; }

  uint32_t get_state_id(uint32_t node_handle) const { return node_states_[node_handle]; }
  uint32_t get_node_handle(uint32_t state_id) const { return lookup_table_[state_id]; }
  NodePool& get_pool() { return pool_; }
  const NodePool& get_pool() const { return pool_; }

private:

  uint32_t w_;
  uint32_t h_;
  uint32_t goal_id_;

  NodePool pool_;
  std::vector<uint32_t> node_states_; // node_handle -> state_id
  std::vector<uint32_t> lookup_table_; // state_id -> node_handle
  std::vector<uint8_t> grid_map_; // state_id -> cost
};

class PancakeEnvironment {

public:

  using T = std::array<uint8_t, 48>;

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

  void get_successors(uint32_t node_index, std::vector<uint32_t>& neighbors);
  uint32_t get_heuristic(const T& state) const;
  bool is_goal(const T& state) const { return state == goal_state_; };

  uint32_t get_start_node();

  T get_state(uint32_t node_handle) const { return node_states_[node_handle]; }
  uint32_t get_node_handle(const T& state) const { 
    auto it = lookup_table_.find(state);
    if (it != lookup_table_.end()) {
      return it->second;
    }
    return NODE_NULL;
  }

  NodePool& get_pool() { return pool_; }
  const NodePool& get_pool() const { return pool_; }

private:

  uint32_t capacity_;
  NodePool pool_;

  std::vector<T> node_states_; // node_handle -> state_id
  std::unordered_map<T, uint32_t, StateHash> lookup_table_;

  T start_state_;
  T goal_state_;

};

// class SlidingTileEnvironment {

// public:

//   void get_successors(uint32_t node_index, std::vector<uint32_t>& neighbors);
//   uint32_t get_heuristic(uint32_t node_index);
//   bool is_goal(uint32_t node_index);

//   uint32_t get_start_node();

//   NodePool& pool;

//   std::vector<uint64_t> node_states; 

// };
