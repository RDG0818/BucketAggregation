// include/queues/bucket_heap.h

#pragma once

#include "environments/node.h"
#include "queues/indexed_d_ary_heap.h"
#include "queues/two_level_bucket_queue.h"
#include "utils/utils.h"

// Priority queue combining a TwoLevelBucketQueue (node storage) with an
// IndexedDaryHeap (tracking which primary buckets are non-empty).
//
// The primary heap is keyed by f-bucket index (= raw_f / beta), so each
// entry represents one primary bucket regardless of bucket width. The
// PriorityCalculator functor maps (f_representative, h_representative) -> priority.
//
// When alpha = beta = 1 and use_h_max = false (the defaults), this reduces to
// the original bucket heap with no aggregation.
template <typename PriorityCalculator, typename Compare = std::greater<double>, int D = 2>
class BucketHeap {

public:

  BucketHeap(PriorityCalculator& calculator, uint32_t alpha = 1, uint32_t beta = 1, bool use_h_max = false)
    : calculator_(calculator), alpha_(alpha), beta_(beta), use_h_max_(use_h_max),
      buckets_(alpha, beta) {};

  void set_use_h_max(bool val) { use_h_max_ = val; }

  void push(uint32_t id, uint32_t f, uint32_t h) {
    // The primary heap is keyed by bucket index, not raw f, so that nodes
    // sharing the same primary bucket map to a single heap entry.
    uint32_t f_key = f / beta_;
    uint32_t f_rep = f_key * beta_; // raw f lower bound used to query the bucket

    size_t old_count = buckets_.get_node_count(f_rep);
    uint32_t old_h_min_idx = buckets_.get_h_min(f_rep);

    buckets_.push(id, f, h);

    uint32_t new_h_min_idx = buckets_.get_h_min(f_rep);
    if (old_count == 0 || new_h_min_idx < old_h_min_idx) {
      double priority = calculator_(static_cast<double>(f_rep), h_rep(new_h_min_idx));
      primary_heap_.push(f_key, priority); // handles decrease-key as well
    }
  }

  uint32_t pop() noexcept {
    if (primary_heap_.empty()) return NODE_NULL;
    uint32_t best_f_key = primary_heap_.top();
    uint32_t best_f_rep = best_f_key * beta_;

    uint32_t old_h_min_idx = buckets_.get_h_min(best_f_rep);
    uint32_t node_id = buckets_.pop_from(best_f_rep);

    if (buckets_.get_node_count(best_f_rep) == 0) {
      primary_heap_.remove(best_f_key);
    }
    else {
      uint32_t new_h_min_idx = buckets_.get_h_min(best_f_rep);
      if (new_h_min_idx != old_h_min_idx) {
        double priority = calculator_(static_cast<double>(best_f_rep), h_rep(new_h_min_idx));
        primary_heap_.change_priority(best_f_key, priority);
      }
    }

    return node_id;
  }

  bool empty() const noexcept {
    return buckets_.empty();
  }

  void clear() noexcept {
    buckets_.clear();
    primary_heap_.clear();
  }

  // Recompute all primary heap priorities from the current state of the bucket
  // queue. Called by ANA* and DPS whenever the priority function changes.
  void rebuild() noexcept {
    auto update_func = [&](uint32_t f_key) {
      uint32_t f_rep = f_key * beta_;
      return calculator_(static_cast<double>(f_rep), h_rep(buckets_.get_h_min(f_rep)));
    };
    primary_heap_.rebuild(update_func);
  }

  PriorityCalculator& get_calculator() noexcept {
    return calculator_;
  }

  // Raw f lower bound of the minimum non-empty primary bucket. Used by DPS to
  // detect when f_min has increased and a rebuild is needed.
  uint32_t get_f_min_raw() const noexcept { return buckets_.get_f_min() * beta_; }

  uint64_t get_hmin_scans() const noexcept { return buckets_.get_hmin_scans(); }
  uint64_t get_secondary_bucket_allocs() const noexcept { return buckets_.get_secondary_bucket_allocs(); }

  template<typename Validator>
  utils::QueueDetailedMetrics get_detailed_metrics(Validator&& v) const {
    return buckets_.get_detailed_metrics(std::forward<Validator>(v));
  }

  utils::QueueDetailedMetrics get_detailed_metrics() const {
    return buckets_.get_detailed_metrics();
  }

private:
  // Returns the representative h-cost for a secondary bucket index. When
  // use_h_max is false (default), uses the lower bound of the bucket interval
  // to preserve admissibility guarantees. When use_h_max is true, uses the
  // upper bound, which can yield higher priorities at the cost of some slack.
  double h_rep(uint32_t h_idx) const noexcept {
    if (use_h_max_) {
      return static_cast<double>((h_idx + 1) * alpha_ - 1);
    }
    return static_cast<double>(h_idx * alpha_);
  }

  TwoLevelBucketQueue buckets_;
  IndexedDaryHeap<double, D, Compare> primary_heap_;
  PriorityCalculator& calculator_;
  uint32_t alpha_;
  uint32_t beta_;
  bool use_h_max_;
};
