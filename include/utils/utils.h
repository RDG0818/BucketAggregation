// include/utils/utils.h

#pragma once

#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <cstdint>
#include <type_traits> // For std::void_t
#include <iomanip>
#include <map>

#ifdef __linux__
#include <fstream>
#include <unistd.h>
#endif

#include "environments/node.h"

namespace utils {

struct QueueDetailedMetrics {
  uint64_t expansions = 0;
  size_t primary_buckets_total = 0;
  size_t primary_buckets_nonempty = 0;
  size_t secondary_buckets_total = 0;
  size_t secondary_buckets_nonempty = 0;
  
  size_t logical_nodes_total = 0;
  size_t logical_primary_nonempty = 0;
  size_t logical_secondary_nonempty = 0;

  uint32_t f_min = 0, f_max = 0;
  
  double h_min_mean = 0, h_min_stddev = 0;
  double h_max_mean = 0, h_max_stddev = 0;
  double sec_per_pri_mean = 0, sec_per_pri_stddev = 0;
  double nodes_per_sec_mean = 0, nodes_per_sec_stddev = 0;

  std::map<uint32_t, size_t> h_distribution; // h_value -> node count

  static void write_csv_header(std::ostream& out) {
    out << "expansions,primary_total,primary_nonempty,secondary_total,secondary_nonempty,"
        << "logical_nodes,logical_primary_nonempty,logical_secondary_nonempty,"
        << "f_min,f_max,h_min_mean,h_min_stddev,h_max_mean,h_max_stddev,"
        << "sec_per_pri_mean,sec_per_pri_stddev,nodes_per_sec_mean,nodes_per_sec_stddev\n";
  }

  void write_csv_row(std::ostream& out) const {
    out << expansions << "," << primary_buckets_total << "," << primary_buckets_nonempty << ","
        << secondary_buckets_total << "," << secondary_buckets_nonempty << ","
        << logical_nodes_total << "," << logical_primary_nonempty << "," << logical_secondary_nonempty << ","
        << f_min << "," << f_max << "," 
        << h_min_mean << "," << h_min_stddev << ","
        << h_max_mean << "," << h_max_stddev << ","
        << sec_per_pri_mean << "," << sec_per_pri_stddev << ","
        << nodes_per_sec_mean << "," << nodes_per_sec_stddev << "\n";
  }
};

struct SearchStats {
  double time_enqueue = 0;
  double time_dequeue = 0;
  double time_rebuild = 0; // For ANA* heap rebuilding
  double time_decrease_key = 0;
  
  uint64_t count_enqueue = 0;
  uint64_t count_dequeue = 0;
  uint64_t count_rebuild = 0;
  uint64_t count_decrease_key = 0;

  uint64_t count_stale_pops = 0;
  uint64_t count_update_pushes = 0;

  uint64_t total_hmin_scans = 0;
  uint64_t total_secondary_bucket_allocs = 0;

  double total_time_ms = 0;
  uint64_t nodes_expanded = 0;
  uint64_t nodes_generated = 0;
  
  double solution_cost = -1.0;
  size_t solution_length = 0;
  
  long memory_peak_kb = 0;

  void reset() { *this = SearchStats(); }
};

inline long get_peak_memory_kb() {
#ifdef __linux__
  std::ifstream status_file("/proc/self/status");
  std::string line;
  while (std::getline(status_file, line)) {
    if (line.substr(0, 6) == "VmHWM:") {
      // Format: "VmHWM:    1234 kB"
      size_t start = line.find_first_of("0123456789");
      size_t end = line.find(" kB");
      if (start != std::string::npos && end != std::string::npos) {
        return std::stol(line.substr(start, end - start));
      }
    }
  }
#endif
  return 0; 
}

// SFINAE helpers
template<typename Q, typename F, typename = std::void_t<>>
struct has_rebuild : std::false_type {};

template<typename Q, typename F>
struct has_rebuild<Q, F, std::void_t<decltype(std::declval<Q>().rebuild(std::declval<F>()))>> : std::true_type {};

template<typename Q, typename = std::void_t<>>
struct has_rebuild_no_args : std::false_type {};

template<typename Q>
struct has_rebuild_no_args<Q, std::void_t<decltype(std::declval<Q>().rebuild())>> : std::true_type {};

template<typename Q, typename = std::void_t<>>
struct has_get_detailed_metrics : std::false_type {};

template<typename Q>
struct has_get_detailed_metrics<Q, std::void_t<decltype(std::declval<Q>().get_detailed_metrics())>> : std::true_type {};

template<typename Q, typename = std::void_t<>>
struct has_get_hmin_scans : std::false_type {};

template<typename Q>
struct has_get_hmin_scans<Q, std::void_t<decltype(std::declval<Q>().get_hmin_scans())>> : std::true_type {};

template<typename Q, typename = std::void_t<>>
struct has_get_secondary_bucket_allocs : std::false_type {};

template<typename Q>
struct has_get_secondary_bucket_allocs<Q, std::void_t<decltype(std::declval<Q>().get_secondary_bucket_allocs())>> : std::true_type {};

template<typename Q, typename = std::void_t<>>
struct has_get_f_min_raw : std::false_type {};

template<typename Q>
struct has_get_f_min_raw<Q, std::void_t<decltype(std::declval<Q>().get_f_min_raw())>> : std::true_type {};

template<typename Q, typename V, typename = std::void_t<>>
struct has_get_detailed_metrics_v : std::false_type {};

template<typename Q, typename V>
struct has_get_detailed_metrics_v<Q, V, std::void_t<decltype(std::declval<Q>().get_detailed_metrics(std::declval<V>()))>> : std::true_type {};

template<typename Q, typename = std::void_t<>>
struct has_contains : std::false_type {};

template<typename Q>
struct has_contains<Q, std::void_t<decltype(std::declval<Q>().contains(uint32_t{}))>> : std::true_type {};

template <typename QueueType>
class ProfiledQueue {
public:
  ProfiledQueue(QueueType& underlying_queue, SearchStats& stats) 
    : queue_(underlying_queue), stats_(stats) {}

  // Matches generic push(id, f, h) interface
  template<typename PriorityType>
  void push(uint32_t handle, PriorityType priority, uint32_t h = 0) {
    bool is_update = false;
    if constexpr (has_contains<QueueType>::value) {
      is_update = queue_.contains(handle);
    }

    auto start = std::chrono::steady_clock::now();
    queue_.push(handle, priority, h);
    auto end = std::chrono::steady_clock::now();
    
    double duration = std::chrono::duration<double, std::nano>(end - start).count();
    if (is_update) {
      stats_.time_decrease_key += duration;
      stats_.count_decrease_key++;
    } else {
      stats_.time_enqueue += duration;
      stats_.count_enqueue++;
    }
  }

  uint32_t pop() {
    auto start = std::chrono::steady_clock::now();
    
    uint32_t handle = queue_.pop();
    
    auto end = std::chrono::steady_clock::now();
    stats_.time_dequeue += std::chrono::duration<double, std::nano>(end - start).count();
    stats_.count_dequeue++;

    if constexpr (has_get_hmin_scans<QueueType>::value) {
      stats_.total_hmin_scans = queue_.get_hmin_scans();
    }
    if constexpr (has_get_secondary_bucket_allocs<QueueType>::value) {
      stats_.total_secondary_bucket_allocs = queue_.get_secondary_bucket_allocs();
    }
    return handle;
  }

  uint32_t top() { return queue_.top(); }
  auto top_priority() { return queue_.top_priority(); }
  void remove(uint32_t id) { queue_.remove(id); }
  void change_priority(uint32_t id, auto p, uint32_t h = 0) { queue_.change_priority(id, p, h); }
  bool contains(uint32_t id) const { return queue_.contains(id); }
  
  uint32_t get_f_min() const { return queue_.get_f_min(); }
  uint32_t get_f_min_raw() const { 
    if constexpr (has_get_f_min_raw<QueueType>::value) {
      return queue_.get_f_min_raw(); 
    }
    return INF_COST;
  }
  uint32_t get_alpha() const { return queue_.get_alpha(); }
  uint32_t get_beta() const { return queue_.get_beta(); }
  
  uint32_t pop_from(uint32_t f) { 
    auto start = std::chrono::steady_clock::now();
    uint32_t handle = queue_.pop_from(f);
    auto end = std::chrono::steady_clock::now();
    stats_.time_dequeue += std::chrono::duration<double, std::nano>(end - start).count();
    stats_.count_dequeue++;

    if constexpr (has_get_hmin_scans<QueueType>::value) {
      stats_.total_hmin_scans = queue_.get_hmin_scans();
    }
    if constexpr (has_get_secondary_bucket_allocs<QueueType>::value) {
      stats_.total_secondary_bucket_allocs = queue_.get_secondary_bucket_allocs();
    }
    return handle; 
  }
  size_t get_node_count(uint32_t f) const { return queue_.get_node_count(f); }
  uint32_t get_h_min(uint32_t f) const { return queue_.get_h_min(f); }

  // ANA* Rebuild support
  template<typename Func>
  void rebuild(Func calculate_priority) {
    if constexpr (has_rebuild<QueueType, Func>::value) {
      auto start = std::chrono::steady_clock::now();
      
      queue_.rebuild(calculate_priority);
      
      auto end = std::chrono::steady_clock::now();
      stats_.time_rebuild += std::chrono::duration<double, std::nano>(end - start).count();
      stats_.count_rebuild++;

      if constexpr (has_get_hmin_scans<QueueType>::value) {
        stats_.total_hmin_scans = queue_.get_hmin_scans();
      }
      if constexpr (has_get_secondary_bucket_allocs<QueueType>::value) {
        stats_.total_secondary_bucket_allocs = queue_.get_secondary_bucket_allocs();
      }
    }
  }

  void rebuild() {
    if constexpr (has_rebuild_no_args<QueueType>::value) {
      auto start = std::chrono::steady_clock::now();
      
      queue_.rebuild();
      
      auto end = std::chrono::steady_clock::now();
      stats_.time_rebuild += std::chrono::duration<double, std::nano>(end - start).count();
      stats_.count_rebuild++;

      if constexpr (has_get_hmin_scans<QueueType>::value) {
        stats_.total_hmin_scans = queue_.get_hmin_scans();
      }
      if constexpr (has_get_secondary_bucket_allocs<QueueType>::value) {
        stats_.total_secondary_bucket_allocs = queue_.get_secondary_bucket_allocs();
      }
    }
  }

  auto& get_calculator() {
    return queue_.get_calculator();
  }

  template<typename Validator>
  utils::QueueDetailedMetrics get_detailed_metrics(Validator&& v) {
    if constexpr (has_get_detailed_metrics_v<QueueType, Validator>::value) {
      return queue_.get_detailed_metrics(std::forward<Validator>(v));
    }
    return utils::QueueDetailedMetrics();
  }

  utils::QueueDetailedMetrics get_detailed_metrics() {
    if constexpr (has_get_detailed_metrics<QueueType>::value) {
      return queue_.get_detailed_metrics();
    }
    return utils::QueueDetailedMetrics();
  }

  bool empty() const { return queue_.empty(); }
  void clear() { queue_.clear(); }

private:
  QueueType& queue_;
  SearchStats& stats_;
};

} // namespace utils