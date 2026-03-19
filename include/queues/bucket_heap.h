// include/queues/bucket_heap.h

#pragma once

#include "environments/node.h"
#include "queues/indexed_d_ary_heap.h"
#include "queues/two_level_bucket_queue.h"
#include "utils/utils.h"

template <typename PriorityCalculator, typename Compare = std::greater<double>, int D = 2>
class BucketHeap {

public:

  BucketHeap(PriorityCalculator& calculator) 
    : calculator_(calculator) {};

  void push(uint32_t id, uint32_t f, uint32_t h) {
    size_t old_count = buckets_.get_node_count(f);
    uint32_t old_h_min = buckets_.get_h_min(f);

    buckets_.push(id, f, h);

    if (old_count == 0 || h < old_h_min) {
      double priority = calculator_(f, h);
      primary_heap_.push(f, priority); // handles decrease-key as well
    }
  }

  uint32_t pop() noexcept {
    if (primary_heap_.empty()) return NODE_NULL;
    uint32_t best_f = primary_heap_.top();
    uint32_t old_h_min = buckets_.get_h_min(best_f);
    uint32_t node_id = buckets_.pop_from(best_f);

    if (buckets_.get_node_count(best_f) == 0) {
      primary_heap_.remove(best_f);
    }
    else {
      uint32_t current_h_min = buckets_.get_h_min(best_f);
      if (current_h_min != old_h_min) {
        double priority = calculator_(best_f, current_h_min);
        primary_heap_.change_priority(best_f, priority);
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

  void rebuild() noexcept {
    auto update_func = [&](uint32_t f) {
      uint32_t h_min = buckets_.get_h_min(f);
      return calculator_(f, h_min);
    };
    primary_heap_.rebuild(update_func);
  }

  PriorityCalculator& get_calculator() {
    return calculator_;
  }

  uint64_t get_hmin_scans() const noexcept { return buckets_.get_hmin_scans(); }
  uint64_t get_secondary_bucket_allocs() const noexcept { return buckets_.get_secondary_bucket_allocs(); }
  uint32_t get_f_min_raw() const noexcept { return buckets_.get_f_min(); }

  template<typename Validator>
  utils::QueueDetailedMetrics get_detailed_metrics(Validator&& v) const {
    return buckets_.get_detailed_metrics(std::forward<Validator>(v));
  }

  utils::QueueDetailedMetrics get_detailed_metrics() const {
    return buckets_.get_detailed_metrics();
  }

private:
  TwoLevelBucketQueue buckets_;
  IndexedDaryHeap<double, D, Compare> primary_heap_;
  PriorityCalculator& calculator_;

}; 