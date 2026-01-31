// include/environments/node.h

#pragma once

#include <cstdint>
#include <limits>
#include <vector>

static constexpr uint32_t NODE_NULL = std::numeric_limits<uint32_t>::max();
static constexpr uint32_t INF_COST = std::numeric_limits<uint32_t>::max();

class NodePool {

public:

  NodePool() = default;

  void prepare_for_search() {
    search_iteration_++;
    if (search_iteration_ == 0) {
      std::fill(generated_at_.begin(), generated_at_.end(), 0);
      std::fill(closed_on_iteration_.begin(), closed_on_iteration_.end(), 0);
      search_iteration_ = 1;
    }
  }

  void reserve(uint32_t capacity) {
    g_costs_.reserve(capacity);
    parents_.reserve(capacity);
    generated_at_.reserve(capacity);
    closed_on_iteration_.reserve(capacity);
  }

  void resize_state_space(uint32_t size) {
    g_costs_.resize(size, INF_COST);
    parents_.resize(size, NODE_NULL);
    generated_at_.resize(size, 0); // 0 means "never generated" relative to search_iteration_ >= 1
    closed_on_iteration_.resize(size, 0);
  }

  uint32_t create_new_state_id() {
    uint32_t id = static_cast<uint32_t>(g_costs_.size());
    g_costs_.push_back(INF_COST);
    parents_.push_back(NODE_NULL);
    generated_at_.push_back(0);
    closed_on_iteration_.push_back(0);
    return id;
  }

  inline bool is_generated(uint32_t id) const {
    return generated_at_[id] == search_iteration_;
  }

  inline void mark_generated(uint32_t id) {
    generated_at_[id] = search_iteration_;

    g_costs_[id] = INF_COST;
    parents_[id] = NODE_NULL;
  }

  inline bool is_closed(uint32_t id) const {
    if (id >= closed_on_iteration_.size()) return false;
    return closed_on_iteration_[id] == search_iteration_;
  }

  inline void mark_closed(uint32_t id) {
    if (id >= closed_on_iteration_.size()) resize_state_space(id + 1);
    closed_on_iteration_[id] = search_iteration_;
  }

  inline uint32_t get_g(uint32_t id) const {
    if (generated_at_[id] != search_iteration_) return INF_COST;
    return g_costs_[id];
  }

  inline void set_g(uint32_t id, uint32_t g) {
    if (generated_at_[id] != search_iteration_) mark_generated(id);
    g_costs_[id] = g;
  }

  inline uint32_t get_parent(uint32_t id) const {
    if (generated_at_[id] != search_iteration_) return NODE_NULL;
    return parents_[id];
  }

  inline void set_parent(uint32_t id, uint32_t parent) { parents_[id] = parent; }
  
  uint32_t size() const { return static_cast<uint32_t>(g_costs_.size()); }

private:
  std::vector<uint32_t> g_costs_;
  std::vector<uint32_t> parents_;

  std::vector<uint16_t> generated_at_;
  std::vector<uint32_t> closed_on_iteration_;
  uint32_t search_iteration_ = 0;

};

