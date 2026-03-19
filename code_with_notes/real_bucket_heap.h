// include/queues/bucket_heap.h

#pragma once

#include "environments/node.h"
#include "queues/indexed_d_ary_heap.h"
#include "queues/two_level_bucket_queue.h"
#include "utils/utils.h"

template <typename PriorityCalculator, typename Compare = std::greater<double>, int D = 2>
class RealBucketHeap {

public:

  // NOTE: Although it would reduce your flexibility a little, if you move alpha and beta to be
  // template parameters, it will give the compiler full insight into their values, so it may be able
  // to better optimize the division operations in push(). Integer division is the slowest CPU operation, and
  // the compiler will avoid it at all costs - if it knows that it can, which requires knowing the values at
  // compilation time.
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

  // NOTE: Mark noexcept
  uint32_t pop() noexcept {
    if (primary_heap_.empty()) return NODE_NULL;
    uint32_t best_f_idx = primary_heap_.top();
    uint32_t old_h_min_idx = buckets_.get_h_min(best_f_idx);
    uint32_t node_id = buckets_.pop_from(best_f_idx);

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

  // NOTE: Mark noexcept
  bool empty() const noexcept {
    return buckets_.empty();
  }

  // NOTE: Mark noexcept
  void clear() noexcept {
    buckets_.clear();
    primary_heap_.clear();
  }

  // NOTE: Mark noexcept
  void rebuild() noexcept {
    auto update_func = [&](uint32_t f_idx) {
      double f_lower = static_cast<double>(f_idx * beta_);
      double h_lower = static_cast<double>(buckets_.get_h_min(f_idx) * alpha_);
      uint32_t h_min = buckets_.get_h_min(f_idx);
      return calculator_(f_lower, h_lower);
    };
    primary_heap_.rebuild(update_func);
  }

  // NOTE: Mark noexcept
  // I'm assuming this is used to modify some member of the calculator externally, otherwise
  // I would recommend returning a const reference and marking the function const as well.
  PriorityCalculator& get_calculator() noexcept {
    return calculator_;
  }

  template<typename Validator>
  utils::QueueDetailedMetrics get_detailed_metrics(Validator&& v) const {
    auto m = buckets_.get_detailed_metrics(std::forward<Validator>(v));
    m.h_min_mean *= alpha_;
    m.h_min_stddev *= alpha_;
    m.h_max_mean *= alpha_;
    m.h_max_stddev *= alpha_;
    
    // Also scale the distribution map
    std::map<uint32_t, size_t> scaled_dist;
    for (auto const& [h_idx, count] : m.h_distribution) {
      scaled_dist[h_idx * alpha_] = count;
    }
    m.h_distribution = std::move(scaled_dist);
    
    return m;
  }

  utils::QueueDetailedMetrics get_detailed_metrics() const {
    auto is_stale_stub = [](uint32_t, uint32_t, uint32_t) { return false; };
    return get_detailed_metrics(is_stale_stub);
  }

private:
  TwoLevelBucketQueue buckets_;
  IndexedDaryHeap<double, D, Compare> primary_heap_;
  PriorityCalculator& calculator_;
  uint32_t alpha_;
  uint32_t beta_;

};