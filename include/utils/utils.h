// include/utils/utils.h

#pragma once

#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <cstdint>
#include <type_traits> // For std::void_t
#include <iomanip>

#ifdef __linux__
#include <fstream>
#include <unistd.h>
#endif

namespace utils {

struct SearchStats {
  double time_enqueue = 0;
  double time_dequeue = 0;
  double time_rebuild = 0; // For ANA* heap rebuilding
  
  uint64_t count_enqueue = 0;
  uint64_t count_dequeue = 0;
  uint64_t count_rebuild = 0;

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

// SFINAE helper to check if Q has rebuild method
template<typename Q, typename F, typename = std::void_t<>>
struct has_rebuild : std::false_type {};

template<typename Q, typename F>
struct has_rebuild<Q, F, std::void_t<decltype(std::declval<Q>().rebuild(std::declval<F>()))>> : std::true_type {};

template<typename Q, typename = std::void_t<>>
struct has_rebuild_no_args : std::false_type {};

template<typename Q>
struct has_rebuild_no_args<Q, std::void_t<decltype(std::declval<Q>().rebuild())>> : std::true_type {};

template <typename QueueType>
class ProfiledQueue {
public:
  ProfiledQueue(QueueType& underlying_queue, SearchStats& stats) 
    : queue_(underlying_queue), stats_(stats) {}

  // Matches generic push(id, f, h) interface
  template<typename PriorityType>
  void push(uint32_t handle, PriorityType priority, uint32_t h = 0) {
    auto start = std::chrono::steady_clock::now();
    
    queue_.push(handle, priority, h);
    
    auto end = std::chrono::steady_clock::now();
    stats_.time_enqueue += std::chrono::duration<double, std::nano>(end - start).count();
    stats_.count_enqueue++;
  }

  uint32_t pop() {
    auto start = std::chrono::steady_clock::now();
    
    uint32_t handle = queue_.pop();
    
    auto end = std::chrono::steady_clock::now();
    stats_.time_dequeue += std::chrono::duration<double, std::nano>(end - start).count();
    stats_.count_dequeue++;
    return handle;
  }

  // ANA* Rebuild support
  template<typename Func>
  void rebuild(Func calculate_priority) {
    if constexpr (has_rebuild<QueueType, Func>::value) {
      auto start = std::chrono::steady_clock::now();
      
      queue_.rebuild(calculate_priority);
      
      auto end = std::chrono::steady_clock::now();
      stats_.time_rebuild += std::chrono::duration<double, std::nano>(end - start).count();
      stats_.count_rebuild++;
    }
  }

  void rebuild() {
    if constexpr (has_rebuild_no_args<QueueType>::value) {
      auto start = std::chrono::steady_clock::now();
      
      queue_.rebuild();
      
      auto end = std::chrono::steady_clock::now();
      stats_.time_rebuild += std::chrono::duration<double, std::nano>(end - start).count();
      stats_.count_rebuild++;
    }
  }

  auto& get_calculator() {
    return queue_.get_calculator();
  }

  bool empty() const { return queue_.empty(); }
  void clear() { queue_.clear(); }

private:
  QueueType& queue_;
  SearchStats& stats_;
};

} // namespace utils