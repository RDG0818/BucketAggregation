// src/sliding_tile_environment.cpp

#include "environments/environments.h"


SlidingTileEnvironment::SlidingTileEnvironment(int instance_index, std::string filename, uint32_t capacity) : capacity_(capacity) {

  goal_state_ = 0x123456789ABCDEF0;

  std::ifstream file(filename);
  if (!file.is_open()) {
    throw std::runtime_error("Could not open benchmark file: " + filename);
  }

  std::string line;
  for (int i = 0; i <= instance_index; i++) {
    if (!std::getline(file, line)) {
      throw std::runtime_error("Instance index out of bounds in file.");
    }
  }

  std::stringstream ss(line);
  int tile_val;
  start_state_ = 0;

  for (int i = 0; i < 16; i++) {
    ss >> tile_val;
    if (ss.fail()) { throw std::runtime_error("Failed to parse tile value."); };
    T val_64 = static_cast<T>(tile_val);
    start_state_ |= (val_64 << ((15 - i) * 4));
  }

  pool_.reserve(capacity_);
  node_states_.reserve(capacity_);
  lookup_table_.reserve(capacity_);
}

uint32_t SlidingTileEnvironment::get_start_node() {
  uint32_t h_val = get_heuristic(start_state_);
  uint32_t handle = pool_.allocate(NODE_NULL, 0, h_val);

  node_states_.push_back(start_state_);
  lookup_table_[start_state_] = handle;

  return handle;
}

uint32_t SlidingTileEnvironment::get_heuristic(T state) const { // Manhattan Distance
  uint32_t h = 0;

  for (int i = 0; i < 16; i++) {
    int tile = get_tile(state, i);
    if (tile == 0) continue;

    int cx = i % 4;
    int cy = i / 4;

    int target_idx = tile - 1;
    int tx = target_idx % 4;
    int ty = target_idx / 4;

    h += std::abs(cx - tx) + std::abs(cy - ty);
  }
  return h;
}

void SlidingTileEnvironment::get_successors(uint32_t parent_handle, std::vector<uint32_t>& neighbors) {
  const uint32_t parent_g = pool_[parent_handle].g;
  const T parent_state = node_states_[parent_handle];
  
  int blank_idx = -1;
  for (int i = 0; i < 16; i++) {
    if (get_tile(parent_state, i) == 0) {
      blank_idx = i;
      break;
    }
  }

  static const int moves[4] = {-4, 4, -1, 1};

  for (int move : moves) {
    int neighbor_idx = blank_idx + move;

    // Bound checks
    if (neighbor_idx < 0 || neighbor_idx >= 16) continue; // vertical
    if (move == -1 && (blank_idx % 4 == 0)) continue; // left
    if (move == 1 && (blank_idx % 4 == 3)) continue; // right

    uint64_t tile_val = get_tile(parent_state, neighbor_idx);
    int shift_blank = (15 - blank_idx) * 4;
    int shift_neighbor = (15 - neighbor_idx) * 4;

    uint64_t swap_mask = (tile_val << shift_blank) | (tile_val << shift_neighbor);

    T n_state = parent_state ^ swap_mask;

    uint32_t new_g = parent_g + 1;
    auto it = lookup_table_.find(n_state);
    if (it == lookup_table_.end()) {
      // NEW NODE
      uint32_t h_val = get_heuristic(n_state);
      uint32_t n_handle = pool_.allocate(parent_handle, new_g, h_val);

      lookup_table_.insert({n_state, n_handle});
      node_states_.push_back(n_state);
      neighbors.push_back(n_handle);
    }
    else {
      // EXISTING NODE
      uint32_t n_handle = it->second;
      if (new_g < pool_[n_handle].g) {
        pool_[n_handle].g = new_g;
        pool_[n_handle].parent = parent_handle;
        neighbors.push_back(n_handle);
      }
    }
    
  }
}