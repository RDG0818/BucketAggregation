#include "environments/environments.h"
#include <algorithm>
#include <random>

int MSAEnvironment::aa_to_idx(char c) {
    if (c == '-') return 20;
    static const char* order = "ARNDCQEGHILKMFPSTWYV";
    c = toupper(c);
    for (int i = 0; i < 20; ++i) {
        if (order[i] == c) return i;
    }
    return 20; // Default to gap for unknown characters
}

MSAEnvironment::MSAEnvironment(const std::vector<std::string>& sequences, uint32_t capacity)
  : capacity_(capacity) {
  
  static const int PAM250_RAW[20][20] = {
    /* A   R   N   D   C   Q   E   G   H   I   L   K   M   F   P   S   T   W   Y   V */
    {  2, -2,  0,  0, -2,  0,  0,  1, -1, -1, -2, -1, -1, -3,  1,  1,  1, -6, -3,  0}, // A
    { -2,  6,  0, -1, -4,  1, -1, -3,  2, -2, -3,  3,  0, -4,  0,  0, -1,  2, -4, -2}, // R
    {  0,  0,  2,  2, -4,  1,  1,  0,  2, -2, -3,  1, -2, -3,  0,  1,  0, -4, -2, -2}, // N
    {  0, -1,  2,  4, -5,  2,  3,  1,  1, -2, -4,  0, -3, -6, -1,  0,  0, -7, -4, -2}, // D
    { -2, -4, -4, -5, 12, -5, -5, -3, -3, -2, -6, -5, -5, -4, -3,  0, -2, -8,  0, -2}, // C
    {  0,  1,  1,  2, -5,  4,  2, -1,  3, -2, -2,  1, -1, -5,  0, -1, -1, -5, -4, -2}, // Q
    {  0, -1,  1,  3, -5,  2,  4,  0,  1, -2, -3,  0, -2, -5, -1,  0,  0, -7, -4, -2}, // E
    {  1, -3,  0,  1, -3, -1,  0,  5, -2, -3, -4, -2, -3, -5,  0,  1,  0, -7, -5, -1}, // G
    { -1,  2,  2,  1, -3,  3,  1, -2,  6, -2, -2,  0, -2, -2,  0, -1, -1, -3,  0, -2}, // H
    { -1, -2, -2, -2, -2, -2, -2, -3, -2,  5,  2, -2,  2,  1, -2, -1,  0, -5, -1,  4}, // I
    { -2, -3, -3, -4, -6, -2, -3, -4, -2,  2,  6, -3,  4,  2, -3, -3, -2, -2, -1,  2}, // L
    { -1,  3,  1,  0, -5,  1,  0, -2,  0, -2, -3,  5,  0, -5, -1,  0,  0, -3, -4, -2}, // K
    { -1,  0, -2, -3, -5, -1, -2, -3, -2,  2,  4,  0,  6,  0, -2, -2, -1, -4, -2,  2}, // M
    { -3, -4, -3, -6, -4, -5, -5, -5, -2,  1,  2, -5,  0,  9, -5, -3, -3,  0,  7, -1}, // F
    {  1,  0,  0, -1, -3,  0, -1,  0,  0, -2, -3, -1, -2, -5,  6,  1,  0, -6, -5, -1}, // P
    {  1,  0,  1,  0,  0, -1,  0,  1, -1, -1, -3,  0, -2, -3,  1,  2,  1, -2, -3, -1}, // S
    {  1, -1,  0,  0, -2, -1,  0,  0, -1,  0, -2,  0, -1, -3,  0,  1,  3, -5, -3,  0}, // T
    { -6,  2, -4, -7, -8, -5, -7, -7, -3, -5, -2, -3, -4,  0, -6, -2, -5, 17,  0, -6}, // W
    { -3, -4, -2, -4,  0, -4, -4, -5,  0, -1, -1, -4, -2,  7, -5, -3, -3,  0, 10, -2}, // Y
    {  0, -2, -2, -2, -2, -2, -2, -1, -2,  4,  2, -2,  2, -1, -1, -1,  0, -6, -2,  4}  // V
  };

  int offset = 17;
  int gap_penalty = 8;

  for (int i = 0; i < 20; ++i) {
    for (int j = 0; j < 20; ++j) {
        cost_table_[i][j] = -PAM250_RAW[i][j] + offset;
    }
    cost_table_[i][20] = gap_penalty + offset;
    cost_table_[20][i] = gap_penalty + offset;
  }
  cost_table_[20][20] = 0 + offset;

  seq_lengths_.resize(N);
  encoded_sequences_.resize(N);
  for (size_t i = 0; i < N; ++i) {
    seq_lengths_[i] = sequences[i].length();
    start_state_[i] = 0;
    goal_state_[i] = seq_lengths_[i];
    
    encoded_sequences_[i].reserve(seq_lengths_[i]);
    for (char c : sequences[i]) {
        encoded_sequences_[i].push_back(static_cast<uint8_t>(aa_to_idx(c)));
    }
  }

  pool_.reserve(capacity);
  node_states_.reserve(capacity);
  lookup_table_.reserve(capacity);

  goal_id_ = get_or_create_id(goal_state_);
  get_or_create_id(start_state_);

  compute_pairwise_heuristics();
}

uint32_t MSAEnvironment::get_start_node() {
  return get_or_create_id(start_state_);
}

uint32_t MSAEnvironment::get_or_create_id(const T& state) {
  auto it = lookup_table_.find(state);
  if (it != lookup_table_.end()) {
    return it->second;
  }
  uint32_t new_id = pool_.create_new_state_id();
  node_states_.push_back(state);
  lookup_table_.insert({state, new_id});
  return new_id;
}

void MSAEnvironment::get_successors(uint32_t u_id, std::vector<uint32_t>& neighbors) {
  neighbors.clear();
  const T current_state = node_states_[u_id];

  uint32_t num_moves = (1 << N) - 1;
  for (uint32_t m = 1; m <= num_moves; ++m) {
    bool valid = true;
    T next_state = current_state;
    for (size_t i = 0; i < N; ++i) {
      if ((m >> i) & 1) {
        if (next_state[i] < (unsigned)seq_lengths_[i]) {
          next_state[i]++;
        } else {
          valid = false;
          break;
        }
      }
    }
    if (valid) {
      neighbors.push_back(get_or_create_id(next_state));
    }
  }
}

uint32_t MSAEnvironment::compute_transition_cost(const T& u_state, const T& v_state) const {
  uint32_t cost = 0;
  uint8_t indices[N];
  for (size_t i = 0; i < N; ++i) {
    if (v_state[i] > u_state[i]) {
      indices[i] = encoded_sequences_[i][u_state[i]];
    } else {
      indices[i] = 20; // GAP
    }
  }

  for (size_t i = 0; i < N; ++i) {
    for (size_t j = i + 1; j < N; ++j) {
      cost += cost_table_[indices[i]][indices[j]];
    }
  }
  return cost;
}

uint32_t MSAEnvironment::get_edge_cost(uint32_t u, uint32_t v) const {
  return compute_transition_cost(node_states_[u], node_states_[v]);
}

bool MSAEnvironment::is_goal(uint32_t id) const {
  return id == goal_id_;
}

uint32_t MSAEnvironment::get_heuristic(uint32_t id) const {
  const T& state = node_states_[id];
  uint32_t h = 0;
  for (size_t i = 0; i < N; ++i) {
    for (size_t j = i + 1; j < N; ++j) {
        if (i < h_tables_.size() && j-i-1 < h_tables_[i].size()) {
            const auto& table = h_tables_[i][j-i-1];
            if (state[i] * (seq_lengths_[j] + 1) + state[j] < table.size()) {
                 h += table[state[i] * (seq_lengths_[j] + 1) + state[j]];
            }
        }
    }
  }
  return h;
}

void MSAEnvironment::compute_pairwise_heuristics() {
  h_tables_.assign(N, std::vector<std::vector<uint32_t>>());
  for(size_t i = 0; i < N; ++i) {
      h_tables_[i].resize(N - 1 - i);
  }

  for (size_t i = 0; i < N; ++i) {
    for (size_t j = i + 1; j < N; ++j) {
      size_t len_i = seq_lengths_[i];
      size_t len_j = seq_lengths_[j];
      auto& table = h_tables_[i][j-i-1];
      table.assign((len_i + 1) * (len_j + 1), 0);

      auto get_idx = [&](size_t x, size_t y) { return x * (len_j + 1) + y; };

      for (int x = (int)len_i - 1; x >= 0; --x) {
          table[get_idx(x, len_j)] = table[get_idx(x + 1, len_j)] + cost_table_[encoded_sequences_[i][x]][20];
      }
      for (int y = (int)len_j - 1; y >= 0; --y) {
          table[get_idx(len_i, y)] = table[get_idx(len_i, y+1)] + cost_table_[20][encoded_sequences_[j][y]];
      }

      for (int x = (int)len_i - 1; x >= 0; --x) {
        for (int y = (int)len_j - 1; y >= 0; --y) {
          uint32_t cost_match = cost_table_[encoded_sequences_[i][x]][encoded_sequences_[j][y]] + table[get_idx(x + 1, y + 1)];
          uint32_t cost_gap_i = cost_table_[encoded_sequences_[i][x]][20] + table[get_idx(x + 1, y)];
          uint32_t cost_gap_j = cost_table_[20][encoded_sequences_[j][y]] + table[get_idx(x, y + 1)];
          table[get_idx(x, y)] = std::min({cost_match, cost_gap_i, cost_gap_j});
        }
      }
    }
  }
}
