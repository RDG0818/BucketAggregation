// include/queues/bucket_heap.h

#pragma once

#include "environments/node.h"
#include "queues/indexed_binary_heap.h"
#include "queues/two_level_bucket_queue.h"
#include "utils/utils.h"

template <typename PriorityCalculator, typename Compare = std::greater<double>>
class RealBucketHeap {

public:

  RealBucketHeap(PriorityCalculator& calculator, uint32_t alpha = 1, uint32_t beta = 1) 
    : calculator_(calculator), alpha_(alpha), beta_(beta) {};

  void push(uint32_t id, uint32_t f, uint32_t h) {

    uint32_t f_idx = f / beta_;
    uint32_t h_idx = h / alpha_;
    size_t old_count = buckets_.get_node_count(f_idx);
    uint32_t old_h_min_idx = buckets_.get_h_min(f_idx);

    buckets_.push(id, f_idx, h_idx);

    if (old_count == 0 || h_idx < old_h_min_idx) {
      double f_lower = static_cast<double>(f_idx * beta_);
      double h_lower = static_cast<double>(buckets_.get_h_min(f_idx) * alpha_);
      double priority = calculator_(f_lower, h_lower);
      primary_heap_.push(f_idx, priority); // handles decrease-key as well
    }
  }

  uint32_t pop() {
    if (primary_heap_.empty()) return NODE_NULL;
    uint32_t best_f_idx = primary_heap_.top();
    uint32_t old_h_min_idx = buckets_.get_h_min(best_f_idx);
    uint32_t node_id = buckets_.pop_from(best_f_idx);
    if (node_id == NODE_NULL) {
    std::cout << "CRITICAL ERROR: Heap thinks bucket " << best_f_idx 
              << " is valid, but TwoLevelBucketQueue returned NODE_NULL!" << std::endl;
    exit(1);
}

    if (buckets_.get_node_count(best_f_idx) == 0) {
      primary_heap_.remove(best_f_idx);
    }
    else {
      uint32_t current_h_min_idx = buckets_.get_h_min(best_f_idx);
      if (current_h_min_idx != old_h_min_idx) {
        double f_lower = static_cast<double>(best_f_idx * beta_);
        double h_lower = static_cast<double>(current_h_min_idx * alpha_);
        double priority = calculator_(f_lower, h_lower);
        primary_heap_.change_priority(best_f_idx, priority);
      }
    }

    return node_id;
  }

  bool empty() const {
    return buckets_.empty();
  }

  void clear() {
    buckets_.clear();
    primary_heap_.clear();
  }

  void rebuild() {
    auto update_func = [&](uint32_t f_idx) {
      double f_lower = static_cast<double>(f_idx * beta_);
      double h_lower = static_cast<double>(buckets_.get_h_min(f_idx) * alpha_);
      uint32_t h_min = buckets_.get_h_min(f_idx);
      return calculator_(f_lower, h_lower);
    };
    primary_heap_.rebuild(update_func);
  }

  PriorityCalculator& get_calculator() {
    return calculator_;
  }

private:
  TwoLevelBucketQueue buckets_;
  IndexedBinaryHeap<double, Compare> primary_heap_;
  PriorityCalculator& calculator_;
  uint32_t alpha_;
  uint32_t beta_;

};