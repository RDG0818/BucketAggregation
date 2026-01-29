#pragma once

#include "environments/node.h"

#include <algorithm>
#include <vector>


class BucketQueue {

public:

  BucketQueue(uint32_t initial_max_cost = 1024) 
    : min_priority_(std::numeric_limits<uint32_t>::max()), count_(0) {
      buckets_.reserve(initial_max_cost);
    }

  void push(uint32_t id, uint32_t priority, uint32_t h = 0) {

    if (priority >= buckets_.size()) {
      size_t new_size = std::max<size_t>(priority + 1, buckets_.size() * 2);
      buckets_.resize(new_size);
    }

    buckets_[priority].push_back(id);

    if (priority < min_priority_) {
      min_priority_ = priority;
    }

    count_++;
  }

  uint32_t pop() {
    if (count_ == 0) {
      return INF_COST;
    }

    while (min_priority_ < buckets_.size() && buckets_[min_priority_].empty()) {
      min_priority_++;
    }

    if (min_priority_ >= buckets_.size()) {
      count_ = 0;
      return INF_COST;
    }

    uint32_t best_id = buckets_[min_priority_].back();
    buckets_[min_priority_].pop_back();

    count_--;

    return best_id;
  }

  bool empty() const { return count_ == 0; };

  void clear() {
    buckets_.clear();
    count_ = 0;
    min_priority_ = std::numeric_limits<uint32_t>::max();
  }

  template <typename T>
  void rebuild(T t) {
    // API consistency
  }

private:

  std::vector<std::vector<uint32_t>> buckets_;
  uint32_t min_priority_;
  uint32_t count_;

};
