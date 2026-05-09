// include/environments/node.h

#pragma once

#include <cstdint>
#include <limits>
#include <vector>

static constexpr uint32_t NODE_NULL = std::numeric_limits<uint32_t>::max();
static constexpr uint32_t INF_COST  = std::numeric_limits<uint32_t>::max();

// NodePool manages per-node search state for a single environment.
//
// To avoid clearing all arrays between searches, every entry is stamped with
// the iteration counter at which it was last touched. A node is considered
// "generated this search" iff its stamp equals search_iteration_, and "closed"
// iff closed_on_iteration_ equals search_iteration_. When the counter wraps
// around to 0, both arrays are bulk-reset and the counter restarts at 1 —
// so iteration 0 is permanently reserved as the "never touched" sentinel.
//
// Both stamp arrays are uint16_t, so the wrap occurs every 65,535 searches.
// search_iteration_ is also uint16_t to keep comparisons exact with no
// implicit widening that could silently break the stamp logic.

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
    generated_at_.resize(size, 0);
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
    if (id >= generated_at_.size()) return false;
    return generated_at_[id] == search_iteration_;
  }

  // Marks the node as generated for the current search and resets its g-cost
  // and parent. Called lazily by set_g() on first access each search.
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

  inline void unmark_closed(uint32_t id) {
    if (id < closed_on_iteration_.size()) {
      closed_on_iteration_[id] = 0;
    }
  }

  inline uint32_t get_g(uint32_t id) const {
    if (id >= generated_at_.size() || generated_at_[id] != search_iteration_) return INF_COST;
    return g_costs_[id];
  }

  inline void set_g(uint32_t id, uint32_t g) {
    if (generated_at_[id] != search_iteration_) mark_generated(id);
    g_costs_[id] = g;
  }

  inline uint32_t get_parent(uint32_t id) const {
    if (id >= generated_at_.size() || generated_at_[id] != search_iteration_) return NODE_NULL;
    return parents_[id];
  }

  inline void set_parent(uint32_t id, uint32_t parent) { parents_[id] = parent; }

  uint32_t size() const { return static_cast<uint32_t>(g_costs_.size()); }

private:
  std::vector<uint32_t> g_costs_;
  std::vector<uint32_t> parents_;

  std::vector<uint16_t> generated_at_;
  std::vector<uint16_t> closed_on_iteration_;
  uint16_t search_iteration_ = 0;

};
