// src/grid_environment.cpp

#include "environments/environments.h"

GridEnvironment::GridEnvironment(uint32_t w, uint32_t h, uint32_t seed) : w_(w), h_(h), goal_id_(w_ * h_ - 1) {
    uint32_t total_states = w * h;
    grid_map_.resize(total_states);
    pool_.resize_state_space(total_states);
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

void GridEnvironment::get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors) {
  neighbors.clear();

  const int cx = u_id % w_;
  const int cy = u_id / w_;

  static const int dx[4] = {0, 0, -1, 1};
  static const int dy[4] = {-1, 1, 0, 0};

  for (int i = 0; i < 4; i++) {
    int nx = cx + dx[i];
    int ny = cy + dy[i];

    if (nx >= 0 && nx < (int)w_ && ny >= 0 && ny < (int)h_) {
      uint32_t v_id = ny * w_ + nx;

      if (grid_map_[v_id] != 0) { neighbors.push_back(v_id); }
    }
  }
}
