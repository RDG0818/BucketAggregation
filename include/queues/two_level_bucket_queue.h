// include/queues/two_level_bucket_queue.h

#pragma once

#include <limits>
#include <memory>
#include <vector>

#include "environments/node.h"

struct Block {
  static constexpr size_t SIZE = 126; // total size: 126 x 4 + 8 = 512 bytes
  uint32_t elements[SIZE];
  Block* next = nullptr;
};

class BlockPool {

public:

  BlockPool(size_t chunk_size = 1000, size_t initial_capacity = 0) 
  : chunk_size_(chunk_size) {
    if (initial_capacity > 0) {
      reserve(initial_capacity);
    }
  };

  ~BlockPool() = default;

  Block* allocate() {
    if (!free_list_) {
      allocate_chunk();
    }
    Block* block = free_list_;
    free_list_ = free_list_->next;
    block->next = nullptr;
    return block;
  }

  void deallocate(Block* block) {
    block->next = free_list_;
    free_list_ = block;
  }

  void reserve(size_t n_blocks) {
    while (capacity_ < n_blocks) {
      allocate_chunk();
    }
  }

  void clear() {
    free_list_ = nullptr;
    capacity_ = 0;
    chunks_.clear();
  }

private:

  void allocate_chunk() {
    auto chunk = std::make_unique<Block[]>(chunk_size_);

    for (size_t i = 0; i < chunk_size_ - 1; i++) {
      chunk[i].next = &chunk[i + 1];
    }
    chunk[chunk_size_ - 1].next = free_list_;
    free_list_ = &chunk[0];

    chunks_.push_back(std::move(chunk));
    capacity_ += chunk_size_;
  }

  size_t chunk_size_;
  size_t capacity_ = 0;
  Block* free_list_ = nullptr;
  std::vector<std::unique_ptr<Block[]>> chunks_;

};

class TwoLevelBucketQueue {

private:

  struct SecondaryBucket {
    Block* head = nullptr;
    uint32_t top = 0;

    bool empty() const {return head == nullptr; };
  };

  struct PrimaryBucket {
    std::vector<SecondaryBucket> h_buckets;
    uint32_t h_min = INF_COST;
    size_t count = 0;
  };

public:

  TwoLevelBucketQueue(uint32_t f_cap_hint = 1024) 
  : f_min_(INF_COST), count_(0), pool_(1000, 1000) {
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

    auto& s_bucket = p_bucket.h_buckets[h];

    if (!s_bucket.head || s_bucket.top == Block::SIZE) {
      Block* new_block = pool_.allocate();
      new_block->next = s_bucket.head;
      s_bucket.head = new_block;
      s_bucket.top = 0;
    }

    s_bucket.head->elements[s_bucket.top++] = id;

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

    return pop_from_bucket(f_buckets_[f_min_]);
  }

  uint32_t pop_from(uint32_t f) {
    if (f >= f_buckets_.size() || f_buckets_[f].count == 0) {
      return NODE_NULL;
    }

    return pop_from_bucket(f_buckets_[f]);
  }

  size_t get_node_count(uint32_t f) const {
    if (f >= f_buckets_.size()) return 0;
    return f_buckets_[f].count;
  }

  uint32_t get_h_min(uint32_t f) const {
    if (f >= f_buckets_.size()) return INF_COST;
    return f_buckets_[f].h_min;
  }

  uint32_t peek_top_node(uint32_t f, uint32_t h) const {
    if (f >= f_buckets_.size() || h >= f_buckets_[f].h_buckets.size()) {  
      return NODE_NULL;
    }

    const auto& s_bucket = f_buckets_[f].h_buckets[h];
    if (s_bucket.empty()) return NODE_NULL;

    return s_bucket.head->elements[s_bucket.top - 1];
  }

  void set_f_min(uint32_t f) {
    f_min_ = f;
  }

  bool empty() const { return count_ == 0; };

  void clear() {
    f_buckets_.clear();
    f_min_ = INF_COST;
    count_ = 0;
    pool_.clear();
  }

  template <typename T> void rebuild(T) {}

private:

  uint32_t pop_from_bucket(PrimaryBucket& p_bucket) {
    while(p_bucket.h_min < p_bucket.h_buckets.size() &&
          p_bucket.h_buckets[p_bucket.h_min].empty()) {
      p_bucket.h_min++;
    }

    if (p_bucket.h_min >= p_bucket.h_buckets.size()) {
      return NODE_NULL;
    }

    auto& s_bucket = p_bucket.h_buckets[p_bucket.h_min];
    uint32_t id = s_bucket.head->elements[--s_bucket.top];

    if (s_bucket.top == 0) {
      Block* old_head = s_bucket.head;
      s_bucket.head = s_bucket.head->next;
      pool_.deallocate(old_head);
      
      if (s_bucket.head) s_bucket.top = Block::SIZE;
    }
    
    p_bucket.count--;
    count_--;

    if (p_bucket.count > 0 && s_bucket.empty()) {
      while (p_bucket.h_min < p_bucket.h_buckets.size() &&
             p_bucket.h_buckets[p_bucket.h_min].empty()) {
        p_bucket.h_min++;
      };
    }

    return id;
  }

  std::vector<PrimaryBucket> f_buckets_;
  uint32_t f_min_;
  size_t count_;
  BlockPool pool_;
};
