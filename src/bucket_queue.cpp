#include "queues/bucket_queue.h"
#include <limits>

BucketQueue::BucketQueue(NodePool& p, uint32_t initial_capacity)
    : pool_(p), min_bucket_idx_(std::numeric_limits<uint32_t>::max()), count_(0) {
    buckets_.resize(initial_capacity);
}

void BucketQueue::resize_if_needed(uint32_t priority) {
    if (priority >= buckets_.size()) {
        buckets_.resize(priority + 1024); // Grow with some buffer
    }
}

void BucketQueue::push(uint32_t handle, uint32_t priority) {
    resize_if_needed(priority);
    buckets_[priority].push_back(handle);
    pool_[handle].queue_ref = priority;
    count_++;
    if (priority < min_bucket_idx_) {
        min_bucket_idx_ = priority;
    }
}

void BucketQueue::decrease_key(uint32_t handle, uint32_t new_priority) {
    resize_if_needed(new_priority);
    buckets_[new_priority].push_back(handle);
    pool_[handle].queue_ref = new_priority; // Update to new priority
    count_++;
    if (new_priority < min_bucket_idx_) {
        min_bucket_idx_ = new_priority;
    }
}

uint32_t BucketQueue::pop() {
    if (empty()) {
        return NODE_NULL;
    }

    while (min_bucket_idx_ < buckets_.size()) {
        if (buckets_[min_bucket_idx_].empty()) {
            min_bucket_idx_++;
            continue;
        }

        uint32_t handle = buckets_[min_bucket_idx_].back();
        buckets_[min_bucket_idx_].pop_back();
        count_--;

        if (pool_[handle].queue_ref != min_bucket_idx_) {
            continue;
        }
        
        pool_[handle].queue_ref = NODE_NULL;
        return handle;
    }

    return NODE_NULL; // Should be unreachable if empty() is checked
}

bool BucketQueue::contains(uint32_t handle) const {
    return pool_[handle].queue_ref != NODE_NULL;
}

bool BucketQueue::empty() const {
    return count_ == 0;
}
