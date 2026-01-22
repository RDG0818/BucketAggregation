#include "queues/two_level_bucket_queue.h"
#include <limits>

TwoLevelBucketQueue::TwoLevelBucketQueue(NodePool& p)
    : pool_(p),
      top_level_idx_(std::numeric_limits<uint32_t>::max()),
      low_level_idx_(0),
      count_(0) {}

void TwoLevelBucketQueue::push(uint32_t handle, uint32_t priority) {
    const uint32_t high_idx = priority >> LOW_LEVEL_BITS;
    const uint32_t low_idx = priority & LOW_LEVEL_MASK;

    // Resize top-level vector if needed
    if (high_idx >= buckets_.size()) {
        buckets_.resize(high_idx + 1);
    }

    // Allocate low-level bucket array if it doesn't exist
    if (!buckets_[high_idx]) {
        buckets_[high_idx] = std::make_unique<LowLevelBuckets>(LOW_LEVEL_SIZE);
    }

    (*buckets_[high_idx])[low_idx].push_back(handle);
    pool_[handle].queue_ref = priority;
    count_++;

    // Update the top-level index hint if we found a new minimum
    if (high_idx < top_level_idx_) {
        top_level_idx_ = high_idx;
        low_level_idx_ = low_idx;
    } else if (high_idx == top_level_idx_ && low_idx < low_level_idx_) {
        low_level_idx_ = low_idx;
    }
}

void TwoLevelBucketQueue::decrease_key(uint32_t handle, uint32_t new_priority) {
    // Lazy decrease-key: just re-insert the node with the new priority.
    // The old entry will be detected as stale and ignored during pop.
    push(handle, new_priority);
}

uint32_t TwoLevelBucketQueue::pop() {
    if (empty()) {
        return NODE_NULL;
    }

    while (top_level_idx_ < buckets_.size()) {
        // Check if the current top-level bucket is valid and allocated
        if (buckets_[top_level_idx_]) {
            while (low_level_idx_ < LOW_LEVEL_SIZE) {
                if (!(*buckets_[top_level_idx_])[low_level_idx_].empty()) {
                    // Found a non-empty bucket, pop an item
                    uint32_t handle = (*buckets_[top_level_idx_])[low_level_idx_].back();
                    (*buckets_[top_level_idx_])[low_level_idx_].pop_back();
                    count_--;

                    uint32_t priority = (top_level_idx_ << LOW_LEVEL_BITS) | low_level_idx_;

                    // Check if stale. A node is stale if its priority in the NodePool
                    // doesn't match the priority of the bucket it came from.
                    if (pool_[handle].queue_ref != priority) {
                        continue; // Stale node, keep searching
                    }

                    // Valid node found
                    pool_[handle].queue_ref = NODE_NULL;
                    return handle;
                }
                low_level_idx_++;
            }
        }
        // Move to the next top-level bucket
        top_level_idx_++;
        low_level_idx_ = 0;
    }

    return NODE_NULL; // No valid items found
}

bool TwoLevelBucketQueue::contains(uint32_t handle) const {
    return pool_[handle].queue_ref != NODE_NULL;
}

bool TwoLevelBucketQueue::empty() const {
    return count_ == 0;
}
