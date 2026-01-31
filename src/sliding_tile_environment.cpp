// src/sliding_tile_environment.cpp

#include "environments/environments.h"


SlidingTileEnvironment::SlidingTileEnvironment(int instance_index, std::string filename, uint32_t capacity) : capacity_(capacity) {

  goal_state_ = 0x0123456789ABCDEF;

  goal_id_ = get_or_create_id(goal_state_);

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
  return get_or_create_id(start_state_);
}

uint32_t SlidingTileEnvironment::get_or_create_id(T state) {
  auto it = lookup_table_.find(state);
  if (it != lookup_table_.end()) {
    return it->second;
  }

  uint32_t new_id = pool_.create_new_state_id();
  node_states_.push_back(state);
  lookup_table_.insert({state, new_id});
  return new_id;
}

void SlidingTileEnvironment::get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors) {
  neighbors.clear();

  const T parent_state = node_states_[u_id];
  
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

    neighbors.push_back(get_or_create_id(n_state));
  }
}