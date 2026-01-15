#pragma once

#include <array>
#include <random>

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

  void get_successors(uint32_t node_index, std::vector<uint32_t>& neighbors);
  uint32_t get_heuristic(uint32_t node_index);
  bool is_goal(uint32_t node_index);

  uint32_t get_start_node();

  NodePool& pool;

  std::vector<std::array<uint8_t, 48>> node_states; 

};

class SlidingTileEnvironment {

public:

  void get_successors(uint32_t node_index, std::vector<uint32_t>& neighbors);
  uint32_t get_heuristic(uint32_t node_index);
  bool is_goal(uint32_t node_index);

  uint32_t get_start_node();

  NodePool& pool;

  std::vector<uint64_t> node_states; 

};
