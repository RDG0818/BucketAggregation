// include/algorithms/a_star.h

#pragma once

#include <vector>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <iostream>
#include "environments/environments.h"
#include "utils/utils.h"

template <typename E, typename PQ> 
class AStar {

public:

  AStar(E& env, PQ& priority_queue, utils::SearchStats* stats = nullptr, bool collect_metrics = false) 
    : env_(env), priority_queue_(priority_queue), stats_(stats), collect_metrics_(collect_metrics) {
      if (collect_metrics_) {
        metrics_out_.open("bucket_metrics.csv", std::ios::trunc);
        if (metrics_out_.is_open()) {
            utils::QueueDetailedMetrics::write_csv_header(metrics_out_);
        }
      }
    };

  ~AStar() {
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
    priority_queue_.push(start_node, 0 + start_h, start_h);

    std::vector<uint32_t> neighbors;
    neighbors.reserve(16);

    uint64_t expansions_count = 0;
    const uint64_t metric_interval = 1000000;

    while (!priority_queue_.empty()) {
      if (collect_metrics_ && expansions_count % metric_interval == 0 && expansions_count > 0) {
        if constexpr (utils::has_get_detailed_metrics<PQ>::value) {
          auto is_stale = [&](uint32_t id, uint32_t f, uint32_t h) {
              return pool.is_closed(id) || (f - h > pool.get_g(id));
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
      pool.mark_closed(u);

      expansions_count++;
      if (stats_) { stats_->nodes_expanded++; }

      if (env_.is_goal(u)) {
        if (stats_) { stats_->solution_cost = pool.get_g(u); }
        // Can construct path if needed
        return;
      }

      env_.get_successors(u, neighbors);
      uint32_t u_g = pool.get_g(u);

      for (uint32_t v : neighbors) {
        uint32_t cost = env_.get_edge_cost(u, v);
        uint32_t new_g = u_g + cost;

        if (new_g < pool.get_g(v)) {
          if (stats_ && pool.get_g(v) != NODE_NULL) {
            stats_->count_update_pushes++;
          }
          pool.set_g(v, new_g);
          pool.set_parent(v, u);
          uint32_t h = env_.get_heuristic(v);
          priority_queue_.push(v, new_g + h, h);

          if (stats_) stats_->nodes_generated++;
        }
      }
    }
  }

private:

  E& env_;
  PQ& priority_queue_;
  utils::SearchStats* stats_;
  bool collect_metrics_;
  std::ofstream metrics_out_;

};