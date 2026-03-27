// include/algorithms/ana_star.h

#pragma once

#include <vector>
#include <cstdint>
#include <limits>
#include <fstream>
#include "environments/node.h"
#include "utils/utils.h"
#include "queues/bucket_heap.h"

template<typename T>
struct is_bucket_heap : std::false_type {};

template<typename PC, typename C, int D>
struct is_bucket_heap<BucketHeap<PC, C, D>> : std::true_type {};

template<typename Q>
struct is_bucket_heap<utils::ProfiledQueue<Q>> : is_bucket_heap<Q> {};

template <typename E, typename PQ>
class ANAStar {

public:
  ANAStar(E& env, PQ& priority_queue, utils::SearchStats* stats = nullptr, bool collect_metrics = false, double weight = 1.0) 
    : env_(env), priority_queue_(priority_queue), stats_(stats), collect_metrics_(collect_metrics), G_upper_(std::numeric_limits<double>::max()) {
      if constexpr (is_bucket_heap<PQ>::value) {
        priority_queue_.get_calculator().set_g_upper(G_upper_);
      }
      if (collect_metrics_) {
        metrics_out_.open("bucket_metrics.csv", std::ios::trunc);
        if (metrics_out_.is_open()) {
            utils::QueueDetailedMetrics::write_csv_header(metrics_out_);
        }
      }
    };

  ~ANAStar() {
      if (metrics_out_.is_open()) metrics_out_.close();
  }

  void print_detailed_metrics(const utils::QueueDetailedMetrics& m) {
      if (metrics_out_.is_open()) {
          m.write_csv_row(metrics_out_);
          metrics_out_.flush();
      }
  }

  void solve() {
    env_.reset_search();
    auto& pool = env_.get_pool();
    uint32_t start_node = env_.get_start_node();

    uint32_t start_h = env_.get_heuristic(start_node);
    pool.set_g(start_node, 0);

    if constexpr (is_bucket_heap<PQ>::value) {
      priority_queue_.push(start_node, start_h, start_h);
    } else {
      priority_queue_.push(start_node, calculate_e(0, start_h), start_h);
    }

    std::vector<uint32_t> neighbors;
    neighbors.reserve(16);

    uint64_t expansions_count = 0;
    const uint64_t metric_interval = 100000;

    while (!priority_queue_.empty()) {

      if (collect_metrics_ && expansions_count % metric_interval == 0 && expansions_count > 0) {
        if constexpr (utils::has_get_detailed_metrics<PQ>::value) {
          auto is_stale = [&](uint32_t id, uint32_t, uint32_t) {
            // Note: Potential function is complex, but g-cost remains best check
            return pool.is_closed(id);
          };
          auto m = priority_queue_.get_detailed_metrics(is_stale);
          m.expansions = expansions_count;
          print_detailed_metrics(m);
        }
      }

      uint32_t u = priority_queue_.pop();

      if (pool.is_closed(u)) {
        if (stats_) stats_->count_stale_pops++;
        continue;
      }

      uint32_t u_g = pool.get_g(u);
      uint32_t u_h = env_.get_heuristic(u);

      if (static_cast<double>(u_g) + u_h >= G_upper_) {
        continue;
      }
      pool.mark_closed(u);

      expansions_count++;
      if (stats_) stats_->nodes_expanded++;

      if (env_.is_goal(u)) {
        if (u_g < G_upper_) {
          G_upper_ = static_cast<double>(u_g);
          if (stats_) stats_->solution_cost = G_upper_;
          if constexpr (is_bucket_heap<PQ>::value) {
            priority_queue_.get_calculator().set_g_upper(G_upper_);
            priority_queue_.rebuild();
          } else {
            auto calculator = [&](uint32_t id) -> double {
              return calculate_e(pool.get_g(id), env_.get_heuristic(id));
            };
            priority_queue_.rebuild(calculator);
          }
        }
        continue;
      }

      env_.get_successors(u, neighbors);

      for (uint32_t v : neighbors) {
        uint32_t cost = env_.get_edge_cost(u, v);
        uint32_t new_g = u_g + cost;
        uint32_t v_h = env_.get_heuristic(v);

        if (static_cast<double>(new_g) + v_h >= G_upper_) {
          continue;
        }

        if (new_g < pool.get_g(v)) {
          if (stats_ && pool.get_g(v) != NODE_NULL) {
            stats_->count_update_pushes++;
          }
          pool.set_g(v, new_g);
          pool.set_parent(v, u);

          if (pool.is_closed(v)) {
            pool.unmark_closed(v);
          }

          if (stats_) stats_->nodes_generated++;

          if constexpr (is_bucket_heap<PQ>::value) {
            priority_queue_.push(v, new_g + v_h, v_h);
          } else {
            double e_val = calculate_e(new_g, v_h);
            priority_queue_.push(v, e_val, v_h);
          }
        }
      }
    }
  }

private:

  double calculate_e(uint32_t g, uint32_t h) const {
    if (h == 0) {
      return std::numeric_limits<double>::max(); // Max priority for goals
    }
    if (g >= G_upper_) {
      return std::numeric_limits<double>::lowest(); // Pruned
    }
    return (G_upper_ - static_cast<double>(g)) / static_cast<double>(h);
  }

  E& env_;
  PQ& priority_queue_;
  utils::SearchStats* stats_;
  bool collect_metrics_;
  double G_upper_;
  std::ofstream metrics_out_;

};
