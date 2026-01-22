#pragma once

#include "environments/node.h"
#include <vector>
#include <memory>

class TwoLevelBucketQueue {

private:

    using LowLevelBuckets = std::vector<std::vector<uint32_t>>;

public:

    TwoLevelBucketQueue(NodePool& p);
    void push(uint32_t handle, uint32_t priority);
    uint32_t pop();
    void decrease_key(uint32_t handle, uint32_t new_priority);
    bool contains(uint32_t handle) const;
    bool empty() const;

private:
    // Defines the number of bits for the low-level index. 10 bits = 1024 buckets.
    static constexpr int LOW_LEVEL_BITS = 10;
    static constexpr uint32_t LOW_LEVEL_MASK = (1 << LOW_LEVEL_BITS) - 1;
    static constexpr size_t LOW_LEVEL_SIZE = 1 << LOW_LEVEL_BITS;

    NodePool& pool_;
    std::vector<std::unique_ptr<LowLevelBuckets>> buckets_;
    uint32_t top_level_idx_;
    uint32_t low_level_idx_;
    size_t count_;
};
