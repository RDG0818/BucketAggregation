// include/queues/aggregated_two_level_bucket_queue.h

#pragma once

#include <limits>
#include <memory>
#include <vector>
#include <numeric>
#include <cmath>

#include "environments/node.h"
#include "queues/two_level_bucket_queue.h" // For Block and BlockPool

class AggregatedTwoLevelBucketQueue {

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

  AggregatedTwoLevelBucketQueue(uint32_t alpha = 1, uint32_t beta = 1, uint32_t f_cap_hint = 1024) 
  : f_min_(INF_COST), count_(0), pool_(1000, 1000), alpha_(alpha), beta_(beta) {
    f_buckets_.reserve(f_cap_hint);
  }

  void push(uint32_t id, uint32_t f, uint32_t h) {
    uint32_t f_idx = f / beta_;
    uint32_t h_idx = h / alpha_;

    if (f_idx >= f_buckets_.size()) {
      size_t new_size = std::max<size_t>(f_idx + 1, f_buckets_.size() * 1.5);
      f_buckets_.resize(new_size);
    }

    auto& p_bucket = f_buckets_[f_idx];

    if (h_idx >= p_bucket.h_buckets.size()) {
      size_t new_size = std::max<size_t>(h_idx + 1, p_bucket.h_buckets.size() * 1.5);
      p_bucket.h_buckets.resize(new_size);
    }

    auto& s_bucket = p_bucket.h_buckets[h_idx];

    if (!s_bucket.head || s_bucket.top == Block::SIZE) {
      Block* new_block = pool_.allocate();
      new_block->next = s_bucket.head;
      s_bucket.head = new_block;
      s_bucket.top = 0;
    }

    s_bucket.head->elements[s_bucket.top++] = id;

    p_bucket.count++;
    count_++;

    if (h_idx < p_bucket.h_min) p_bucket.h_min = h_idx;
    if (f_idx < f_min_) f_min_ = f_idx;
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

  uint32_t pop_from(uint32_t f_idx) {
    if (f_idx >= f_buckets_.size() || f_buckets_[f_idx].count == 0) {
      return NODE_NULL;
    }
    return pop_from_bucket(f_buckets_[f_idx]);
  }

  bool empty() const { return count_ == 0; };

  uint32_t get_f_min() const {
    while (f_min_ < f_buckets_.size() && f_buckets_[f_min_].count == 0) {
      f_min_++;
    }
    return f_min_;
  }

  uint32_t get_f_min_raw() const {
    return get_f_min() * beta_;
  }

  uint32_t get_alpha() const { return alpha_; }
  uint32_t get_beta() const { return beta_; }

  size_t get_node_count(uint32_t f_idx) const {
    if (f_idx >= f_buckets_.size()) return 0;
    return f_buckets_[f_idx].count;
  }

  uint32_t get_h_min(uint32_t f_idx) const {
    if (f_idx >= f_buckets_.size()) return INF_COST;
    return f_buckets_[f_idx].h_min;
  }

  utils::QueueDetailedMetrics get_detailed_metrics() const {
    utils::QueueDetailedMetrics m;
    m.primary_buckets_total = f_buckets_.size();
    m.f_min = f_min_;
    m.f_max = 0;

    std::vector<uint32_t> h_mins;
    std::vector<uint32_t> h_maxs;
    std::vector<double> sec_per_pri_vals;
    std::vector<size_t> nodes_per_sec_vals;

    for (uint32_t f = 0; f < f_buckets_.size(); ++f) {
      const auto& p_bucket = f_buckets_[f];
      if (p_bucket.count == 0) continue;

      m.primary_buckets_nonempty++;
      m.f_max = f;
      m.secondary_buckets_total += p_bucket.h_buckets.size();

      uint32_t local_h_min = INF_COST;
      uint32_t local_h_max = 0;
      size_t local_sec_nonempty = 0;

      for (uint32_t h = 0; h < p_bucket.h_buckets.size(); ++h) {
        const auto& s_bucket = p_bucket.h_buckets[h];
        if (s_bucket.empty()) continue;

        local_sec_nonempty++;
        m.secondary_buckets_nonempty++;
        if (h < local_h_min) local_h_min = h;
        if (h > local_h_max) local_h_max = h;

        size_t node_count = 0;
        Block* curr = s_bucket.head;
        uint32_t top = s_bucket.top;
        while (curr) {
          node_count += top;
          curr = curr->next;
          top = Block::SIZE;
        }
        m.h_distribution[h * alpha_] += node_count;
        nodes_per_sec_vals.push_back(node_count);
      }

      if (local_h_min != INF_COST) {
        h_mins.push_back(local_h_min * alpha_);
        h_maxs.push_back(local_h_max * alpha_);
        sec_per_pri_vals.push_back(static_cast<double>(local_sec_nonempty));
      }
    }

    auto calculate_stats = [](const auto& v, double& mean, double& stddev) {
      if (v.empty()) {
        mean = 0; stddev = 0;
        return;
      }
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
  mutable uint32_t f_min_;
  size_t count_;
  BlockPool pool_;
  uint32_t alpha_;
  uint32_t beta_;
};
