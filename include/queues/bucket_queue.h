// include/queues/bucket_queue.h

#pragma once

#include "environments/node.h"

#include <algorithm>
#include <vector>


class BucketQueue {

public:

  BucketQueue(uint32_t initial_max_cost = 1024) 
    : priority_offset_(0), min_priority_idx_(INF_COST), count_(0) {
      buckets_.reserve(initial_max_cost);
    }

  void push(uint32_t id, uint32_t priority, uint32_t h = 0) {
    if (count_ == 0) {
      priority_offset_ = priority;
      min_priority_idx_ = 0;
    }

    if (priority < priority_offset_) {
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
    if (count_ == 0) {
      return INF_COST;
    }

    while (min_priority_idx_ < buckets_.size() && buckets_[min_priority_idx_].empty()) {
      min_priority_idx_++;
    }

    if (min_priority_idx_ >= buckets_.size()) {
      count_ = 0;
      return INF_COST;
    }

    uint32_t best_id = buckets_[min_priority_idx_].back();
    buckets_[min_priority_idx_].pop_back();

    count_--;

    return best_id;
  }

  bool empty() const { return count_ == 0; };

  void clear() {
    buckets_.clear();
    count_ = 0;
    priority_offset_ = 0;
    min_priority_idx_ = INF_COST;
  }

  template <typename T>
  void rebuild(T t) {
    // API consistency
  }

private:

  std::vector<std::vector<uint32_t>> buckets_;
  uint32_t priority_offset_;
  uint32_t min_priority_idx_;
  uint32_t count_;

};
