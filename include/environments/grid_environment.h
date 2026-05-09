// include/environments/grid_environment.h

#pragma once

#include <cstdint>
#include <vector>
#include "node.h"

// 4-connected grid with 20% random obstacles and uniform edge costs of 1.
// Goal is the bottom-right cell; heuristic is Manhattan distance.
class GridEnvironment {

public:

  GridEnvironment(uint32_t w, uint32_t h, uint32_t seed = 41);

  void reset_search() { pool_.prepare_for_search(); }

  void get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors);

  inline uint32_t get_heuristic(uint32_t id) const {
    return (w_ - 1 - (id % w_)) + (h_ - 1 - (id / w_)); // Manhattan distance
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
  std::vector<uint8_t> grid_map_; // cell id -> traversal cost (0 = obstacle)
  NodePool pool_;

};
