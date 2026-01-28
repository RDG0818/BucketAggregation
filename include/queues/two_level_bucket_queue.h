#pragma once

#include <limits>
#include <memory>
#include <vector>

#include "environments/node.h"

class TwoLevelBucketQueue {

private:

  using SecondaryBucket = std::vector<uint32_t>; // indexed by h cost

  struct PrimaryBucket {
    std::unique_ptr<std::vector<SecondaryBucket>> h_buckets;
    uint32_t h_min_idx = std::numeric_limits<uint32_t>::max();
  };

public:

  TwoLevelBucketQueue(NodePool& p, uint32_t f_cap, uint32_t h_cap);

  void push(uint32_t handle, uint32_t f_cost, uint32_t h_cost);
  uint32_t pop();
  void decrease_key(uint32_t handle);
  bool contains(uint32_t handle) const { return pool_[handle].queue_ref != NODE_NULL; };
  bool empty() const { return count_ == 0; };

private:
  void resize_f_buckets_if_needed_(uint32_t f_cost);
  void resize_h_buckets_if_needed_(PrimaryBucket& p_bucket, uint32_t h_cost);
  void find_next_f_min_();
  void find_next_h_min_(PrimaryBucket& p_bucket);

  NodePool& pool_;
  std::vector<std::unique_ptr<PrimaryBucket>> f_buckets_;
  uint32_t f_min_idx_;
  uint32_t f_cap_;
  uint32_t h_cap_;
  size_t count_;
};
