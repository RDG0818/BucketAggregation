// src/sliding_tile_environment.cpp

#include "environments/environments.h"

const int8_t SlidingTileEnvironment::MOVES[16][4] = {
    {1, 4, -1, -1},   {0, 2, 5, -1},    {1, 3, 6, -1},    {2, 7, -1, -1},   // 0-3
    {0, 5, 8, -1},    {1, 4, 6, 9},     {2, 5, 7, 10},    {3, 6, 11, -1},   // 4-7
    {4, 9, 12, -1},   {5, 8, 10, 13},   {6, 9, 11, 14},   {7, 10, 15, -1},  // 8-11
    {8, 13, -1, -1},  {9, 12, 14, -1},  {10, 13, 15, -1}, {11, 14, -1, -1}  // 12-15
};

const uint8_t SlidingTileEnvironment::MD_TABLE[16][16] = {
    {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0}, // Tile 0 (Blank) - usually ignored, keeping 0s
    {1,0,1,2, 2,1,2,3, 3,2,3,4, 4,3,4,5}, // Tile 1 (Target: 1)
    {2,1,0,1, 3,2,1,2, 4,3,2,3, 5,4,3,4}, // Tile 2 (Target: 2)
    {3,2,1,0, 4,3,2,1, 5,4,3,2, 6,5,4,3}, // Tile 3 (Target: 3)
    {1,2,3,4, 0,1,2,3, 1,2,3,4, 2,3,4,5}, // Tile 4 (Target: 4)
    {2,1,2,3, 1,0,1,2, 2,1,2,3, 3,2,3,4}, // Tile 5 (Target: 5)
    {3,2,1,2, 2,1,0,1, 3,2,1,2, 4,3,2,3}, // Tile 6 (Target: 6)
    {4,3,2,1, 3,2,1,0, 4,3,2,1, 5,4,3,2}, // Tile 7 (Target: 7)
    {2,3,4,5, 1,2,3,4, 0,1,2,3, 1,2,3,4}, // Tile 8 (Target: 8)
    {3,2,3,4, 2,1,2,3, 1,0,1,2, 2,1,2,3}, // Tile 9 (Target: 9)
    {4,3,2,3, 3,2,1,2, 2,1,0,1, 3,2,1,2}, // Tile 10 (Target: 10)
    {5,4,3,2, 4,3,2,1, 3,2,1,0, 4,3,2,1}, // Tile 11 (Target: 11)
    {3,4,5,6, 2,3,4,5, 1,2,3,4, 0,1,2,3}, // Tile 12 (Target: 12)
    {4,3,4,5, 3,2,3,4, 2,1,2,3, 1,0,1,2}, // Tile 13 (Target: 13)
    {5,4,3,4, 4,3,2,3, 3,2,1,2, 2,1,0,1}, // Tile 14 (Target: 14)
    {6,5,4,3, 5,4,3,2, 4,3,2,1, 3,2,1,0} // Tile 15 (Target: 15)
};

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

  for (int i = 0; i < 4; i++) {
    int neighbor_idx = MOVES[blank_idx][i];

    if (neighbor_idx == -1) break;

    uint64_t tile_val = get_tile(parent_state, neighbor_idx);
    int shift_blank = (15 - blank_idx) * 4;
    int shift_neighbor = (15 - neighbor_idx) * 4;

    uint64_t swap_mask = (tile_val << shift_blank) | (tile_val << shift_neighbor);

    T n_state = parent_state ^ swap_mask;

    neighbors.push_back(get_or_create_id(n_state));
  }
}

const int8_t HeavySlidingTileEnvironment::MOVES[16][4] = {
    {1, 4, -1, -1},   {0, 2, 5, -1},    {1, 3, 6, -1},    {2, 7, -1, -1},   // 0-3
    {0, 5, 8, -1},    {1, 4, 6, 9},     {2, 5, 7, 10},    {3, 6, 11, -1},   // 4-7
    {4, 9, 12, -1},   {5, 8, 10, 13},   {6, 9, 11, 14},   {7, 10, 15, -1},  // 8-11
    {8, 13, -1, -1},  {9, 12, 14, -1},  {10, 13, 15, -1}, {11, 14, -1, -1}  // 12-15
};

// Reusing the same MD table
const uint8_t HeavySlidingTileEnvironment::MD_TABLE[16][16] = {
    {0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0}, 
    {1,0,1,2, 2,1,2,3, 3,2,3,4, 4,3,4,5}, 
    {2,1,0,1, 3,2,1,2, 4,3,2,3, 5,4,3,4}, 
    {3,2,1,0, 4,3,2,1, 5,4,3,2, 6,5,4,3}, 
    {1,2,3,4, 0,1,2,3, 1,2,3,4, 2,3,4,5}, 
    {2,1,2,3, 1,0,1,2, 2,1,2,3, 3,2,3,4}, 
    {3,2,1,2, 2,1,0,1, 3,2,1,2, 4,3,2,3}, 
    {4,3,2,1, 3,2,1,0, 4,3,2,1, 5,4,3,2}, 
    {2,3,4,5, 1,2,3,4, 0,1,2,3, 1,2,3,4}, 
    {3,2,3,4, 2,1,2,3, 1,0,1,2, 2,1,2,3}, 
    {4,3,2,3, 3,2,1,2, 2,1,0,1, 3,2,1,2}, 
    {5,4,3,2, 4,3,2,1, 3,2,1,0, 4,3,2,1}, 
    {3,4,5,6, 2,3,4,5, 1,2,3,4, 0,1,2,3}, 
    {4,3,4,5, 3,2,3,4, 2,1,2,3, 1,0,1,2}, 
    {5,4,3,4, 4,3,2,3, 3,2,1,2, 2,1,0,1}, 
    {6,5,4,3, 5,4,3,2, 4,3,2,1, 3,2,1,0} 
};

HeavySlidingTileEnvironment::HeavySlidingTileEnvironment(int instance_index, std::string filename, uint32_t capacity) : capacity_(capacity) {
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

uint32_t HeavySlidingTileEnvironment::get_start_node() {
  return get_or_create_id(start_state_);
}

uint32_t HeavySlidingTileEnvironment::get_or_create_id(T state) {
  auto it = lookup_table_.find(state);
  if (it != lookup_table_.end()) {
    return it->second;
  }
  uint32_t new_id = pool_.create_new_state_id();
  node_states_.push_back(state);
  lookup_table_.insert({state, new_id});
  return new_id;
}

void HeavySlidingTileEnvironment::get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors) {
  neighbors.clear();
  const T parent_state = node_states_[u_id];
  
  int blank_idx = -1;
  for (int i = 0; i < 16; i++) {
    if (get_tile(parent_state, i) == 0) {
      blank_idx = i;
      break;
    }
  }

  for (int i = 0; i < 4; i++) {
    int neighbor_idx = MOVES[blank_idx][i];
    if (neighbor_idx == -1) break;

    uint64_t tile_val = get_tile(parent_state, neighbor_idx);
    int shift_blank = (15 - blank_idx) * 4;
    int shift_neighbor = (15 - neighbor_idx) * 4;
    uint64_t swap_mask = (tile_val << shift_blank) | (tile_val << shift_neighbor);
    
    T n_state = parent_state ^ swap_mask;
    neighbors.push_back(get_or_create_id(n_state));
  }
}