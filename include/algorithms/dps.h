// include/algorithms/dps.h

#pragma once

#include <vector>
#include <cstdint>
#include <limits>
#include <iostream>
#include <iomanip>
#include <fstream>
#include "environments/node.h"
#include "utils/utils.h"
#include "queues/bucket_heap.h"
#include "queues/real_bucket_heap.h"
#include "queues/log_bucket_heap.h"

// Reuse the bucket heap detection from ANA* context or define it here
template<typename T>
struct is_bucket_heap_dps : std::false_type {};

template<typename PC, typename C, int D>
struct is_bucket_heap_dps<BucketHeap<PC, C, D>> : std::true_type {};

template<typename PC, typename C, int D>
struct is_bucket_heap_dps<RealBucketHeap<PC, C, D>> : std::true_type {};

template<typename PC, typename C, int D>
struct is_bucket_heap_dps<LogBucketHeap<PC, C, D>> : std::true_type {};

template<typename Q>
struct is_bucket_heap_dps<utils::ProfiledQueue<Q>> : is_bucket_heap_dps<Q> {};

template<typename Q, typename = std::void_t<>>
struct has_get_f_min_raw_dps : std::false_type {};

template<typename Q>
struct has_get_f_min_raw_dps<Q, std::void_t<decltype(std::declval<Q>().get_f_min_raw())>> : std::true_type {};

template <typename E, typename PQ>
class DynamicPotentialSearch {

public:
  DynamicPotentialSearch(E& env, PQ& priority_queue, utils::SearchStats* stats = nullptr, bool collect_metrics = false, double epsilon = 1.5) 
    : env_(env), priority_queue_(priority_queue), stats_(stats), collect_metrics_(collect_metrics), epsilon_(epsilon), last_f_min_(0) {
      if (collect_metrics_) {
        metrics_out_.open("bucket_metrics_dps.csv", std::ios::trunc);
        if (metrics_out_.is_open()) {
            utils::QueueDetailedMetrics::write_csv_header(metrics_out_);
        }
      }
    };

  ~DynamicPotentialSearch() {
      if (metrics_out_.is_open()) metrics_out_.close();
  }

  void solve() {
    env_.reset_search();
    auto& pool = env_.get_pool();
    uint32_t start_node = env_.get_start_node();
    uint32_t start_h = env_.get_heuristic(start_node);
    pool.set_g(start_node, 0);

    // Initial f_min is start_h
    last_f_min_ = static_cast<double>(start_h);
    update_calculator_fmin(last_f_min_);

    if constexpr (is_bucket_heap_dps<PQ>::value) {
      priority_queue_.push(start_node, start_h, start_h);
    } else {
      priority_queue_.push(start_node, calculate_ud(0, start_h), start_h);
    }

    std::vector<uint32_t> neighbors;
    neighbors.reserve(16);

    uint64_t expansions_count = 0;

    while (!priority_queue_.empty()) {
      // 1. Check for f_min increase to trigger rebuild
      double current_f_min = last_f_min_;
      if constexpr (has_get_f_min_raw_dps<PQ>::value) {
          current_f_min = static_cast<double>(priority_queue_.get_f_min_raw());
      }

      if (current_f_min > last_f_min_) {
          last_f_min_ = current_f_min;
          update_calculator_fmin(last_f_min_);
          
          if constexpr (is_bucket_heap_dps<PQ>::value) {
              priority_queue_.rebuild();
          } else {
              auto calculator = [&](uint32_t id) -> double {
                  return calculate_ud(pool.get_g(id), env_.get_heuristic(id));
              };
              priority_queue_.rebuild(calculator);
          }
      }

      uint32_t u = priority_queue_.pop();

      if (pool.is_closed(u)) {
        if (stats_) stats_->count_stale_pops++;
        continue;
      }
      pool.mark_closed(u);

      expansions_count++;
      if (stats_) stats_->nodes_expanded++;

      if (env_.is_goal(u)) {
        if (stats_) stats_->solution_cost = static_cast<double>(pool.get_g(u));
        return; 
      }

      env_.get_successors(u, neighbors);
      uint32_t u_g = pool.get_g(u);

      for (uint32_t v : neighbors) {
        uint32_t cost = env_.get_edge_cost(u, v);
        uint32_t new_g = u_g + cost;
        uint32_t v_h = env_.get_heuristic(v);

        if (new_g < pool.get_g(v)) {
          if (stats_ && pool.get_g(v) != NODE_NULL) {
            stats_->count_update_pushes++;
          }
          pool.set_g(v, new_g);
          pool.set_parent(v, u);
          
          if (stats_) stats_->nodes_generated++;
                    
          if constexpr (is_bucket_heap_dps<PQ>::value) {
            priority_queue_.push(v, new_g + v_h, v_h);
          } else {
            priority_queue_.push(v, calculate_ud(new_g, v_h), v_h);
          }
        }
      }
    }
  }

private:

  void update_calculator_fmin(double f_min) {
      if constexpr (is_bucket_heap_dps<PQ>::value) {
          priority_queue_.get_calculator().set_f_min(f_min);
          priority_queue_.get_calculator().set_epsilon(epsilon_);
      }
  }

  double calculate_ud(uint32_t g, uint32_t h) const {
    if (h == 0) return std::numeric_limits<double>::max();
    double bound = epsilon_ * last_f_min_;
    if (static_cast<double>(g) >= bound) return std::numeric_limits<double>::lowest();
    return (bound - static_cast<double>(g)) / static_cast<double>(h);
  }

  E& env_;
  PQ& priority_queue_;
  utils::SearchStats* stats_;
  bool collect_metrics_;
  double epsilon_;
  double last_f_min_;
  std::ofstream metrics_out_;

};
