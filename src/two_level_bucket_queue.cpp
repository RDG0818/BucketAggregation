#include "queues/two_level_bucket_queue.h"
#include <limits>


TwoLevelBucketQueue::TwoLevelBucketQueue(NodePool& p, uint32_t f_cap, uint32_t h_cap) 
  : pool_(p),
    f_min_idx_(std::numeric_limits<uint32_t>::max()),
    f_cap_(f_cap),
    h_cap_(h_cap),
    f_buckets_(f_cap_),
    count_(0) {};