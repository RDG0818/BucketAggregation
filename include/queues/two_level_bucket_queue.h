#pragma once

#include <limits>
#include <memory>
#include <vector>

#include "environments/node.h"

class TwoLevelBucketQueue {

private:

  using SecondaryBucket = std::vector<uint32_t>; 

  struct PrimaryBucket {
    std::vector<std::unique_ptr<SecondaryBucket>> h_buckets;
    uint32_t h_min = std::numeric_limits<uint32_t>::max();
    size_t count = 0;
  };

public:

  TwoLevelBucketQueue(uint32_t f_cap_hint = 1024) 
  : f_min_(INF_COST), count_(0) {
    f_buckets_.reserve(f_cap_hint);
  }

  void push(uint32_t id, uint32_t f, uint32_t h) {
    if (f >= f_buckets_.size()) {
      size_t new_size = std::max<size_t>(f + 1, f_buckets_.size() * 1.5);
      f_buckets_.resize(new_size);
    }

    auto& p_bucket = f_buckets_[f];

    if (h >= p_bucket.h_buckets.size()) {
      size_t new_size = std::max<size_t>(h + 1, p_bucket.h_buckets.size() * 1.5);
      p_bucket.h_buckets.resize(new_size);
    }

    if (!p_bucket.h_buckets[h]) {
      p_bucket.h_buckets[h] = std::make_unique<SecondaryBucket>();
    }

    p_bucket.h_buckets[h]->push_back(id);
    p_bucket.count++;
    count_++;

    if (h < p_bucket.h_min) p_bucket.h_min = h;
    if (f < f_min_) f_min_ = f;
  }

  uint32_t pop() {
    if (count_ == 0) return INF_COST;

    while (f_min_ < f_buckets_.size() && f_buckets_[f_min_].count == 0) {
      f_min_++;
    }

    if (f_min_ >= f_buckets_.size()) {
      count_ = 0;
      return INF_COST;
    }

    auto& p_bucket = f_buckets_[f_min_];

    while (p_bucket.h_min < p_bucket.h_buckets.size() &&
           (!p_bucket.h_buckets[p_bucket.h_min] || p_bucket.h_buckets[p_bucket.h_min]->empty())) {
        p_bucket.h_min++;
    }
    
    uint32_t id = p_bucket.h_buckets[p_bucket.h_min]->back();
    p_bucket.h_buckets[p_bucket.h_min]->pop_back();

    p_bucket.count--;
    count_--;

    return id;
  }

  size_t get_node_count(uint32_t f) const {
    if (f >= f_buckets_.size()) return 0;
    return f_buckets_[f].count;
  }

  uint32_t get_h_min(uint32_t f) const {
    if (f >= f_buckets_.size()) return std::numeric_limits<uint32_t>::max();
    return f_buckets_[f].h_min;
  }

  uint32_t peek_top_node(uint32_t f, uint32_t h) const {
    if (f >= f_buckets_.size() || h >= f_buckets_[f].h_buckets.size() || 
        !f_buckets_[f].h_buckets[h] || f_buckets_[f].h_buckets[h]->empty()) {
      return NODE_NULL;
    }
    return f_buckets_[f].h_buckets[h]->back();
  }

  void set_f_min(uint32_t f) {
    f_min_ = f;
  }

  bool empty() const { return count_ == 0; };

  void clear() {
    f_buckets_.clear();
    f_min_ = INF_COST;
    count_ = 0;
  }

  template <typename T> void rebuild(T) {}

private:
  std::vector<PrimaryBucket> f_buckets_;

  uint32_t f_min_;
  size_t count_;
};
