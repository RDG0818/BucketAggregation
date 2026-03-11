#include "environments/environments.h"
#include <algorithm>
#include <random>

// Define the static constexpr member
constexpr uint32_t MSAEnvironment::GAP_COST;

MSAEnvironment::MSAEnvironment(const std::vector<std::string>& sequences, uint32_t capacity)
  : sequences_(sequences), capacity_(capacity), pam250_scorer_() {
  
  seq_lengths_.resize(N);
  for (size_t i = 0; i < N; ++i) {
    seq_lengths_[i] = sequences[i].length();
    start_state_[i] = 0;
    goal_state_[i] = seq_lengths_[i];
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
  const T& current_state = node_states_[u_id];

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

uint32_t MSAEnvironment::get_pairwise_cost(char a, char b) const {
    // Casting to a non-const version to call a non-const method
    return pam250_scorer_.getCost(a, b);
}

uint32_t MSAEnvironment::compute_transition_cost(const T& u_state, const T& v_state) const {
  uint32_t cost = 0;
  char chars[N];
  for (size_t i = 0; i < N; ++i) {
    if (v_state[i] > u_state[i]) {
      chars[i] = sequences_[i][u_state[i]];
    } else {
      chars[i] = '-';
    }
  }

  for (size_t i = 0; i < N; ++i) {
    for (size_t j = i + 1; j < N; ++j) {
      char ci = chars[i];
      char cj = chars[j];
      if (ci == '-' && cj == '-') {
        // cost is 0
      } else if (ci == '-' || cj == '-') {
        cost += GAP_COST;
      } else {
        cost += get_pairwise_cost(ci, cj);
      }
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

      for (int x = len_i - 1; x >= 0; --x) {
          table[get_idx(x, len_j)] = table[get_idx(x + 1, len_j)] + GAP_COST;
      }
      for (int y = len_j - 1; y >= 0; --y) {
          table[get_idx(len_i, y)] = table[get_idx(len_i, y+1)] + GAP_COST;
      }

      for (int x = (int)len_i - 1; x >= 0; --x) {
        for (int y = (int)len_j - 1; y >= 0; --y) {
          uint32_t cost_match = get_pairwise_cost(sequences_[i][x], sequences_[j][y]) + table[get_idx(x + 1, y + 1)];
          uint32_t cost_gap_i = GAP_COST + table[get_idx(x + 1, y)];
          uint32_t cost_gap_j = GAP_COST + table[get_idx(x, y + 1)];
          table[get_idx(x, y)] = std::min({cost_match, cost_gap_i, cost_gap_j});
        }
      }
    }
  }
}
