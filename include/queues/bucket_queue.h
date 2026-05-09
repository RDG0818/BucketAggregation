// include/queues/bucket_queue.h
//
// Integer-indexed bucket queue with O(1) amortized push and pop.
// Each bucket holds a list of node IDs at that integer priority; pop is LIFO
// within a bucket. A cursor (min_priority_idx_) advances forward on pop and
// resets on push of a lower priority, so each slot is visited at most once
// per monotone phase.
//
// Designed for use with A* on integer-cost graphs. Does not support priority
// updates (rebuild is a no-op — A* never reprioritizes).
//
// Note: pushing a priority below priority_offset_ requires shifting the bucket
// array (O(buckets) insert at front). This is safe for A* where f is
// non-decreasing on the open list, but would degrade otherwise.

#pragma once

#include "environments/node.h"

#include <algorithm>
#include <cassert>
#include <vector>


class BucketQueue {

public:

  BucketQueue(uint32_t initial_max_cost = 1024)
    : priority_offset_(0), min_priority_idx_(INF_COST), count_(0) {
      buckets_.reserve(initial_max_cost);
    }

  // h is accepted for interface uniformity with other queues but is not used.
  void push(uint32_t id, uint32_t priority, uint32_t /*h*/ = 0) {
    if (count_ == 0) {
      priority_offset_ = priority;
      min_priority_idx_ = 0;
    }

    if (priority < priority_offset_) {
      // Prepend empty buckets to accommodate the lower priority.
      // O(buckets_.size()) — expected to be rare in A* where f is non-decreasing.
      uint32_t delta = priority_offset_ - priority;
      buckets_.insert(buckets_.begin(), delta, std::vector<uint32_t>{});
      priority_offset_ = priority;
      min_priority_idx_ = 0;
    }

    uint32_t p_idx = priority - priority_offset_;

    if (p_idx >= buckets_.size()) {
      size_t new_size = std::max<size_t>(p_idx + 1, buckets_.size() * 2);
      buckets_.resize(new_size);
    }

    buckets_[p_idx].push_back(id);

    if (p_idx < min_priority_idx_) {
      min_priority_idx_ = p_idx;
    }

    count_++;
  }

  uint32_t pop() {
    if (count_ == 0) return NODE_NULL;

    while (min_priority_idx_ < buckets_.size() && buckets_[min_priority_idx_].empty()) {
      min_priority_idx_++;
    }

    if (min_priority_idx_ >= buckets_.size()) {
      count_ = 0;
      return NODE_NULL;
    }

    uint32_t best_id = buckets_[min_priority_idx_].back();
    buckets_[min_priority_idx_].pop_back();
    count_--;

    return best_id;
  }

  bool empty() const { return count_ == 0; }

  void clear() {
    buckets_.clear();
    count_ = 0;
    priority_offset_ = 0;
    min_priority_idx_ = INF_COST;
  }

  // BucketQueue does not support priority updates. Present for interface
  // compatibility with A*, which never calls rebuild. Calling this is a bug.
  template <typename T>
  void rebuild(T) {
    assert(false && "BucketQueue::rebuild called — this queue is for A* only.");
  }

private:

  std::vector<std::vector<uint32_t>> buckets_;
  uint32_t priority_offset_;
  uint32_t min_priority_idx_;
  uint32_t count_;

};
