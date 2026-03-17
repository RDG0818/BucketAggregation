// include/algorithms/focal_search.h

#pragma once

#include <vector>
#include <cstdint>
#include <limits>
#include <iostream>
#include <set>
#include "environments/node.h"
#include "utils/utils.h"

// Standard Focal Search using two separate priority queues
template <typename E, typename OpenPQ, typename FocalPQ>
class FocalSearch {

public:
  FocalSearch(E& env, OpenPQ& open_pq, FocalPQ& focal_pq, utils::SearchStats* stats = nullptr, bool collect_metrics = false, double weight = 1.5) 
    : env_(env), open_pq_(open_pq), focal_pq_(focal_pq), stats_(stats), collect_metrics_(collect_metrics), weight_(weight) {}

  void solve() {
    env_.reset_search();
    auto& pool = env_.get_pool();
    uint32_t start_node = env_.get_start_node();
    uint32_t start_h = env_.get_heuristic(start_node);
    
    pool.set_g(start_node, 0);
    open_pq_.push(start_node, start_h, start_h);

    // Multiset to track all f-costs currently in the Focal list to find true f_min
    std::multiset<uint32_t> focal_f_costs;

    std::vector<uint32_t> neighbors;
    neighbors.reserve(16);

    while (!open_pq_.empty() || !focal_pq_.empty()) {
      // 1. Calculate true f_min (minimum f of ALL unexpanded nodes)
      uint32_t true_f_min = INF_COST;
      if (!open_pq_.empty()) true_f_min = open_pq_.top_priority();
      if (!focal_f_costs.empty()) true_f_min = std::min(true_f_min, *focal_f_costs.begin());

      if (true_f_min == INF_COST) break;

      double bound = weight_ * static_cast<double>(true_f_min);

      // 2. Transfer eligible nodes from Open to Focal
      while (!open_pq_.empty() && static_cast<double>(open_pq_.top_priority()) <= bound) {
        uint32_t u_idx = open_pq_.pop();
        uint32_t u_f = pool.get_g(u_idx) + env_.get_heuristic(u_idx);
        uint32_t u_h = env_.get_heuristic(u_idx);
        focal_pq_.push(u_idx, u_h, u_h);
        focal_f_costs.insert(u_f);
      }

      if (focal_pq_.empty()) continue; 

      // 3. Dequeue from Focal (best h)
      uint32_t u = focal_pq_.pop();
      uint32_t u_f = pool.get_g(u) + env_.get_heuristic(u);
      auto it = focal_f_costs.find(u_f);
      if (it != focal_f_costs.end()) focal_f_costs.erase(it);

      if (pool.is_closed(u)) continue;
      pool.mark_closed(u);
      if (stats_) stats_->nodes_expanded++;

      if (env_.is_goal(u)) {
        if (stats_) stats_->solution_cost = pool.get_g(u);
        return;
      }

      env_.get_successors(u, neighbors);
      uint32_t u_g = pool.get_g(u);

      for (uint32_t v : neighbors) {
        uint32_t cost = env_.get_edge_cost(u, v);
        uint32_t new_g = u_g + cost;

        if (new_g < pool.get_g(v)) {
          uint32_t old_g = pool.get_g(v);
          uint32_t v_h = env_.get_heuristic(v);
          uint32_t old_f = (old_g == INF_COST) ? INF_COST : old_g + v_h;
          
          pool.set_g(v, new_g);
          pool.set_parent(v, u);
          uint32_t v_f = new_g + v_h;

          if (focal_pq_.contains(v)) {
              focal_pq_.push(v, v_h, v_h);
              auto it_f = focal_f_costs.find(old_f);
              if (it_f != focal_f_costs.end()) focal_f_costs.erase(it_f);
              focal_f_costs.insert(v_f);
          } else if (open_pq_.contains(v)) {
              open_pq_.push(v, v_f, v_h);
          } else {
              if (static_cast<double>(v_f) <= bound) {
                  focal_pq_.push(v, v_h, v_h);
                  focal_f_costs.insert(v_f);
              } else {
                  open_pq_.push(v, v_f, v_h);
              }
          }
          if (stats_) stats_->nodes_generated++;
        }
      }
    }
  }

private:
  E& env_;
  OpenPQ& open_pq_;
  FocalPQ& focal_pq_;
  utils::SearchStats* stats_;
  bool collect_metrics_;
  double weight_;
};


// Bucket-based Focal Search (Focal Search using a heap of buckets)
template <typename E, typename BucketPQ, typename FocalHeap>
class BucketFocalSearch {

public:
  BucketFocalSearch(E& env, BucketPQ& buckets, FocalHeap& focal_heap, utils::SearchStats* stats = nullptr, bool collect_metrics = false, double weight = 1.5) 
    : env_(env), buckets_(buckets), focal_heap_(focal_heap), stats_(stats), collect_metrics_(collect_metrics), weight_(weight) {}

  void solve() {
    uint32_t beta = buckets_.get_beta();
    env_.reset_search();
    auto& pool = env_.get_pool();
    uint32_t start_node = env_.get_start_node();
    uint32_t start_h = env_.get_heuristic(start_node);
    
    pool.set_g(start_node, 0);
    buckets_.push(start_node, start_h, start_h);

    std::vector<uint32_t> neighbors;
    neighbors.reserve(16);

    uint32_t f_idx_min = buckets_.get_f_min();
    if (f_idx_min == INF_COST) return;

    uint32_t f_min_raw = f_idx_min * beta;
    uint32_t f_bound_idx = static_cast<uint32_t>(weight_ * f_min_raw) / beta;

    update_focal_heap(f_idx_min, f_bound_idx);

    while (!buckets_.empty()) {
      if (focal_heap_.empty()) {
        f_idx_min = buckets_.get_f_min();
        if (f_idx_min == INF_COST) break;
        f_min_raw = f_idx_min * beta;
        f_bound_idx = static_cast<uint32_t>(weight_ * f_min_raw) / beta;
        update_focal_heap(f_idx_min, f_bound_idx);
        if (focal_heap_.empty()) break; 
      }

      uint32_t best_f_idx = focal_heap_.top();
      uint32_t u = buckets_.pop_from(best_f_idx);
      
      if (u == NODE_NULL) {
          focal_heap_.remove(best_f_idx);
          continue;
      }

      if (buckets_.get_node_count(best_f_idx) == 0) {
        focal_heap_.remove(best_f_idx);
      } else {
        uint32_t new_h_min = buckets_.get_h_min(best_f_idx);
        focal_heap_.change_priority(best_f_idx, new_h_min, new_h_min);
      }

      if (pool.is_closed(u)) continue;
      pool.mark_closed(u);
      if (stats_) stats_->nodes_expanded++;

      if (env_.is_goal(u)) {
        if (stats_) stats_->solution_cost = pool.get_g(u);
        return;
      }

      env_.get_successors(u, neighbors);
      uint32_t u_g = pool.get_g(u);

      for (uint32_t v : neighbors) {
        uint32_t cost = env_.get_edge_cost(u, v);
        uint32_t new_g = u_g + cost;

        if (new_g < pool.get_g(v)) {
          pool.set_g(v, new_g);
          pool.set_parent(v, u);
          uint32_t v_h = env_.get_heuristic(v);
          uint32_t v_f = new_g + v_h;
          uint32_t v_f_idx = v_f / beta;

          uint32_t old_h_min = buckets_.get_h_min(v_f_idx);
          buckets_.push(v, v_f, v_h);
          
          if (v_f_idx <= f_bound_idx) {
             uint32_t new_h_min = buckets_.get_h_min(v_f_idx);
             if (!focal_heap_.contains(v_f_idx)) {
                 focal_heap_.push(v_f_idx, new_h_min, new_h_min);
             } else if (new_h_min < old_h_min) {
                 focal_heap_.change_priority(v_f_idx, new_h_min, new_h_min);
             }
          }
          if (stats_) stats_->nodes_generated++;
        }
      }

      uint32_t new_f_idx_min = buckets_.get_f_min();
      if (new_f_idx_min != INF_COST && new_f_idx_min > f_idx_min) {
        f_idx_min = new_f_idx_min;
        f_min_raw = f_idx_min * beta;
        f_bound_idx = static_cast<uint32_t>(weight_ * f_min_raw) / beta;
        update_focal_heap(f_idx_min, f_bound_idx);
      }
    }
  }

private:
  void update_focal_heap(uint32_t f_idx_min, uint32_t f_bound_idx) {
    if (f_idx_min == INF_COST) return;
    uint32_t limit = std::min<uint32_t>(f_bound_idx, f_idx_min + 10000); 
    for (uint32_t f_idx = f_idx_min; f_idx <= limit; ++f_idx) {
      if (buckets_.get_node_count(f_idx) > 0 && !focal_heap_.contains(f_idx)) {
        uint32_t h_min = buckets_.get_h_min(f_idx);
        focal_heap_.push(f_idx, h_min, h_min);
      }
    }
  }

  E& env_;
  BucketPQ& buckets_;
  FocalHeap& focal_heap_;
  utils::SearchStats* stats_;
  bool collect_metrics_;
  double weight_;
};
