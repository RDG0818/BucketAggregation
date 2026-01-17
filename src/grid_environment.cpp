// src/grid_environment.cpp

#include "environments/environments.h"

GridEnvironment::GridEnvironment(uint32_t w, uint32_t h, uint32_t seed) : w_(w), h_(h), goal_id_(w_ * h_ - 1) {
    uint32_t total_states = w * h;
    pool_.reserve(total_states);
    node_states_.reserve(total_states);
    lookup_table_.resize(total_states, NODE_NULL);
    grid_map_.resize(total_states);

    std::mt19937 gen(seed);
    std::uniform_real_distribution<> obs_dist(0.0, 1.0); 
    std::uniform_int_distribution<> cost_dist(1, 10);    

    float obstacle_pct = 0.20f;

    for (uint32_t i = 0; i < total_states; ++i) {
      if (obs_dist(gen) < obstacle_pct) {
        grid_map_[i] = 0;
      } else {
        grid_map_[i] = static_cast<uint8_t>(cost_dist(gen));
      }
    }

    grid_map_[0] = 1;
    grid_map_[goal_id_] = 1;
  };

void GridEnvironment::get_successors(uint32_t parent_handle, std::vector<uint32_t>& neighbors) {
  const uint32_t parent_g = pool_[parent_handle].g;
  const uint32_t parent_state_id = node_states_[parent_handle];

  const int cx = parent_state_id % w_;
  const int cy = parent_state_id / w_;

  static const int dx[4] = {0, 0, -1, 1};
  static const int dy[4] = {-1, 1, 0, 0};

  for (int i = 0; i < 4; i++) {
    int nx = cx + dx[i];
    int ny = cy + dy[i];

    if (nx >= 0 && nx < w_ && ny >= 0 && ny < h_) {
      uint32_t n_state_id = ny * w_ + nx;
      uint8_t move_cost = grid_map_[n_state_id];
      if (move_cost == 0) continue;
      uint32_t new_g = parent_g + move_cost; 
      uint32_t n_handle = lookup_table_[n_state_id];

      if (n_handle == NODE_NULL) {
       // New node
        uint32_t h_val = get_heuristic(n_state_id);
        n_handle = pool_.allocate(parent_handle, new_g, h_val);

        lookup_table_[n_state_id] = n_handle;
        node_states_.push_back(n_state_id);
        neighbors.push_back(n_handle);
      }
      else if (new_g < pool_[n_handle].g) {
        // Existing node + shorter path
        pool_[n_handle].g = new_g;
        pool_[n_handle].parent = parent_handle;
        
        neighbors.push_back(n_handle);
      }
    }
  }
  return;
}
