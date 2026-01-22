#pragma once

#include "environments/node.h"
#include <vector>

class BucketQueue {

public:

    BucketQueue(NodePool& p, uint32_t initial_capacity = 4096);
    void push(uint32_t handle, uint32_t priority);
    uint32_t pop();
    void decrease_key(uint32_t handle, uint32_t new_priority);
    bool contains(uint32_t handle) const;
    bool empty() const;

private:

    void resize_if_needed(uint32_t priority);

    NodePool& pool_;
    std::vector<std::vector<uint32_t>> buckets_;
    uint32_t min_bucket_idx_;
    uint32_t count_;
};
