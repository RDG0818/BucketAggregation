// include/queues/two_level_bucket_queue.h
//
// Two-level bucket queue indexed by f-cost (primary) and h-cost (secondary).
// Secondary buckets store node IDs in 512-byte blocks managed by a pool
// allocator, avoiding per-node heap allocation.
//
// When alpha > 1 or beta > 1, secondary/primary buckets are aggregated: a node
// with cost f is placed in primary bucket floor(f/beta), and a node with cost h
// is placed in secondary bucket floor(h/alpha). This reduces bucket sparsity in
// domains with large optimal f-cost and informative heuristics, at the expense
// of some resolution in h-ordering. With alpha=beta=1 (the defaults), this
// behaves as a standard two-level bucket queue.
//
// Used as the node-storage layer inside BucketHeap; BucketHeap manages
// priorities externally via IndexedDaryHeap, so rebuild() is a no-op here.

#pragma once

#include <limits>
#include <memory>
#include <vector>
#include <numeric>
#include <cmath>
#include <cassert>

#include "environments/node.h"
#include "utils/utils.h"

class TwoLevelBucketQueue {

private:

  // 512-byte block for storing node IDs in a secondary bucket.
  // Blocks are linked into a stack: push to head, pop from head.
  struct Block {
    static constexpr size_t SIZE = 126; // 126×4 + 8 = 512 bytes
    uint32_t elements[SIZE];
    Block* next = nullptr;
  };

  // Slab allocator for Block objects. Allocates in chunks to amortize
  // system allocation cost; recycles freed blocks via a free list.
  class BlockPool {
  public:

    BlockPool(size_t chunk_size = 1000, size_t initial_capacity = 0)
    : chunk_size_(chunk_size) {
      if (initial_capacity > 0) reserve(initial_capacity);
    }

    Block* allocate() {
      if (!free_list_) allocate_chunk();
      Block* block = free_list_;
      free_list_ = free_list_->next;
      block->next = nullptr;
      return block;
    }

    void deallocate(Block* block) noexcept {
      block->next = free_list_;
      free_list_ = block;
    }

    void reserve(size_t n_blocks) {
      while (capacity_ < n_blocks) allocate_chunk();
    }

    void clear() noexcept {
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

  struct SecondaryBucket {
    Block* head = nullptr;
    uint32_t top = 0;
    bool empty() const noexcept { return head == nullptr; }
  };

  struct PrimaryBucket {
    std::vector<SecondaryBucket> h_buckets;
    uint32_t h_min = INF_COST; // index of minimum non-empty secondary bucket
    size_t count = 0;
  };

public:

  TwoLevelBucketQueue(uint32_t alpha = 1, uint32_t beta = 1, uint32_t f_cap_hint = 1024)
  : f_offset_(0), f_min_idx_(INF_COST), count_(0), pool_(1000, 1000),
    alpha_(alpha), beta_(beta) {
    f_buckets_.reserve(f_cap_hint);
  }

  void push(uint32_t id, uint32_t f, uint32_t h) {
    uint32_t f_b = f / beta_;
    uint32_t h_b = h / alpha_;

    if (count_ == 0) {
      f_offset_ = f_b;
      f_min_idx_ = 0;
    }

    if (f_b < f_offset_) [[unlikely]] {
      // Prepend empty primary buckets to accommodate the lower f-cost.
      // Should not occur for A*-family algorithms where f is non-decreasing on
      // the open list; this path is O(buckets) due to the front insert.
      uint32_t delta = f_offset_ - f_b;
      f_buckets_.insert(f_buckets_.begin(), delta, PrimaryBucket{});
      f_offset_ = f_b;
      f_min_idx_ = 0;
    }

    uint32_t f_idx = f_b - f_offset_;
    if (f_idx >= f_buckets_.size()) {
      size_t new_size = std::max<size_t>(f_idx + 1, f_buckets_.size() + f_buckets_.size() / 2);
      f_buckets_.resize(new_size);
    }

    auto& p_bucket = f_buckets_[f_idx];

    if (h_b >= p_bucket.h_buckets.size()) {
      if (p_bucket.h_buckets.capacity() == 0) {
        secondary_bucket_allocs_++;
      }
      size_t new_size = std::max<size_t>(h_b + 1, p_bucket.h_buckets.size() + p_bucket.h_buckets.size() / 2);
      p_bucket.h_buckets.resize(new_size);
    }

    auto& s_bucket = p_bucket.h_buckets[h_b];

    if (!s_bucket.head || s_bucket.top == Block::SIZE) {
      Block* new_block = pool_.allocate();
      new_block->next = s_bucket.head;
      s_bucket.head = new_block;
      s_bucket.top = 0;
    }

    s_bucket.head->elements[s_bucket.top++] = id;

    p_bucket.count++;
    count_++;

    if (h_b < p_bucket.h_min) p_bucket.h_min = h_b;
    if (f_idx < f_min_idx_) f_min_idx_ = f_idx;
  }

  uint32_t pop() noexcept {
    if (count_ == 0) return NODE_NULL;

    while (f_min_idx_ < f_buckets_.size() && f_buckets_[f_min_idx_].count == 0) {
      f_min_idx_++;
    }

    if (f_min_idx_ >= f_buckets_.size()) {
      count_ = 0;
      return NODE_NULL;
    }

    return pop_from_bucket(f_buckets_[f_min_idx_]);
  }

  // Pop a node from the primary bucket containing raw cost f.
  uint32_t pop_from(uint32_t f) noexcept {
    uint32_t f_b = f / beta_;
    if (f_b < f_offset_) return NODE_NULL;
    uint32_t f_idx = f_b - f_offset_;
    if (f_idx >= f_buckets_.size() || f_buckets_[f_idx].count == 0) {
      return NODE_NULL;
    }
    return pop_from_bucket(f_buckets_[f_idx]);
  }

  size_t get_node_count(uint32_t f) const noexcept {
    uint32_t f_b = f / beta_;
    if (f_b < f_offset_) return 0;
    uint32_t f_idx = f_b - f_offset_;
    if (f_idx >= f_buckets_.size()) return 0;
    return f_buckets_[f_idx].count;
  }

  // Returns the index of the minimum non-empty secondary bucket for primary
  // bucket f. With alpha > 1 this is a bucket index, not a raw h-cost.
  uint32_t get_h_min(uint32_t f) const noexcept {
    uint32_t f_b = f / beta_;
    if (f_b < f_offset_) return INF_COST;
    uint32_t f_idx = f_b - f_offset_;
    if (f_idx >= f_buckets_.size()) return INF_COST;
    return f_buckets_[f_idx].h_min;
  }

  // Returns the minimum non-empty primary bucket index (= raw f_min / beta).
  // f_min_idx_ is mutable so this lazy cursor-advance can be const.
  uint32_t get_f_min() const noexcept {
    while (f_min_idx_ < f_buckets_.size() && f_buckets_[f_min_idx_].count == 0) {
      f_min_idx_++;
    }
    if (f_min_idx_ >= f_buckets_.size()) return f_offset_ + f_buckets_.size();
    return f_offset_ + f_min_idx_;
  }

  // Returns the raw f lower bound of the minimum non-empty primary bucket.
  uint32_t get_f_min_raw() const noexcept { return get_f_min() * beta_; }

  uint32_t get_alpha() const noexcept { return alpha_; }
  uint32_t get_beta() const noexcept { return beta_; }

  uint64_t get_hmin_scans() const noexcept { return hmin_scans_; }
  uint64_t get_secondary_bucket_allocs() const noexcept { return secondary_bucket_allocs_; }

  bool empty() const noexcept { return count_ == 0; }

  void clear() noexcept {
    f_buckets_.clear();
    f_offset_ = 0;
    f_min_idx_ = INF_COST;
    count_ = 0;
    hmin_scans_ = 0;
    secondary_bucket_allocs_ = 0;
    pool_.clear();
  }

  template <typename Validator>
  utils::QueueDetailedMetrics get_detailed_metrics(Validator&& is_stale) const {
    utils::QueueDetailedMetrics m;
    m.primary_buckets_total = f_buckets_.size();
    m.f_min = get_f_min();
    m.f_max = f_offset_ + f_buckets_.size() - 1;

    std::vector<uint32_t> h_mins;
    std::vector<uint32_t> h_maxs;
    std::vector<double> sec_per_pri_vals;
    std::vector<size_t> nodes_per_sec_vals;

    for (uint32_t f_idx = 0; f_idx < f_buckets_.size(); ++f_idx) {
      const auto& p_bucket = f_buckets_[f_idx];
      uint32_t f_val = (f_offset_ + f_idx) * beta_;
      if (p_bucket.count == 0) continue;

      m.primary_buckets_nonempty++;
      m.f_max = f_offset_ + f_idx;
      m.secondary_buckets_total += p_bucket.h_buckets.size();

      uint32_t local_h_min = INF_COST;
      uint32_t local_h_max = 0;
      size_t local_sec_nonempty = 0;
      bool p_bucket_has_logical = false;

      for (uint32_t h_idx = 0; h_idx < p_bucket.h_buckets.size(); ++h_idx) {
        const auto& s_bucket = p_bucket.h_buckets[h_idx];
        if (s_bucket.empty()) continue;

        local_sec_nonempty++;
        m.secondary_buckets_nonempty++;
        if (h_idx < local_h_min) local_h_min = h_idx;
        if (h_idx > local_h_max) local_h_max = h_idx;

        size_t node_count = 0;
        size_t logical_node_count = 0;
        Block* curr = s_bucket.head;
        uint32_t top = s_bucket.top;
        while (curr) {
          for (uint32_t i = 0; i < top; ++i) {
            if (!is_stale(curr->elements[i], f_val, h_idx * alpha_)) {
              logical_node_count++;
            }
          }
          node_count += top;
          curr = curr->next;
          top = Block::SIZE;
        }

        if (logical_node_count > 0) {
          m.logical_secondary_nonempty++;
          m.logical_nodes_total += logical_node_count;
          p_bucket_has_logical = true;
        }

        m.h_distribution[h_idx * alpha_] += node_count;
        nodes_per_sec_vals.push_back(node_count);
      }

      if (p_bucket_has_logical) {
        m.logical_primary_nonempty++;
      }

      if (local_h_min != INF_COST) {
        h_mins.push_back(local_h_min * alpha_);
        h_maxs.push_back(local_h_max * alpha_);
        sec_per_pri_vals.push_back(static_cast<double>(local_sec_nonempty));
      }
    }

    auto calculate_stats = [](const auto& v, double& mean, double& stddev) {
      if (v.empty()) { mean = 0; stddev = 0; return; }
      double sum = std::accumulate(v.begin(), v.end(), 0.0);
      mean = sum / v.size();
      double sq_sum = std::inner_product(v.begin(), v.end(), v.begin(), 0.0);
      stddev = std::sqrt(std::max(0.0, sq_sum / v.size() - mean * mean));
    };

    calculate_stats(h_mins, m.h_min_mean, m.h_min_stddev);
    calculate_stats(h_maxs, m.h_max_mean, m.h_max_stddev);
    calculate_stats(sec_per_pri_vals, m.sec_per_pri_mean, m.sec_per_pri_stddev);
    calculate_stats(nodes_per_sec_vals, m.nodes_per_sec_mean, m.nodes_per_sec_stddev);

    return m;
  }

  utils::QueueDetailedMetrics get_detailed_metrics() const {
    auto is_stale_stub = [](uint32_t, uint32_t, uint32_t) { return false; };
    return get_detailed_metrics(is_stale_stub);
  }

  // Priorities are managed externally by BucketHeap via IndexedDaryHeap.
  // Calling rebuild on TwoLevelBucketQueue directly is a bug.
  template <typename T>
  void rebuild(T) {
    assert(false && "TwoLevelBucketQueue::rebuild called — priorities are managed by BucketHeap.");
  }

private:

  uint32_t pop_from_bucket(PrimaryBucket& p_bucket) noexcept {
    // Advance h_min cursor past any empty secondary buckets.
    while (p_bucket.h_min < p_bucket.h_buckets.size() &&
           p_bucket.h_buckets[p_bucket.h_min].empty()) {
      p_bucket.h_min++;
      hmin_scans_++;
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

    // If the current secondary bucket is now empty, advance h_min so the
    // next get_h_min() call returns an up-to-date value without scanning.
    if (p_bucket.count > 0 && s_bucket.empty()) {
      while (p_bucket.h_min < p_bucket.h_buckets.size() &&
             p_bucket.h_buckets[p_bucket.h_min].empty()) {
        p_bucket.h_min++;
        hmin_scans_++;
      }
    }

    return id;
  }

  std::vector<PrimaryBucket> f_buckets_;
  uint32_t f_offset_;
  mutable uint32_t f_min_idx_; // mutable: lazily advanced by get_f_min() and pop()
  size_t count_;
  uint64_t hmin_scans_ = 0;
  uint64_t secondary_bucket_allocs_ = 0;
  BlockPool pool_;
  uint32_t alpha_;
  uint32_t beta_;
};
