#pragma once

#include <chrono>
#include <functional>
#include <iostream>
#include <string>
#include <sstream>
#include <utility>
#include <vector>
#include <cstdint>
#include <type_traits> // Required for SFINAE

#ifdef __linux__
#include <fstream>
#include <unistd.h>
#endif

namespace utils {

// A comprehensive struct to hold statistics from a search algorithm's execution.
struct SearchStats {
    // 1. Queue Micro-Benchmarks (in nanoseconds)
    double time_enqueue = 0;
    double time_dequeue = 0;
    double time_change_priority = 0;
    double time_reorder = 0; // Specific to ANA* rebuild
    
    uint64_t count_enqueue = 0;
    uint64_t count_dequeue = 0;
    uint64_t count_change_priority = 0;
    uint64_t count_reorder = 0;

    // 2. Search Macro-Metrics
    double total_time_ms = 0;
    uint64_t nodes_expanded = 0;
    uint64_t nodes_generated = 0;
    uint64_t nodes_reopened = 0;
    double solution_cost = -1;
    size_t solution_length = 0;
    double suboptimality_bound = -1;
    
    // 3. Bucket Heap Specifics
    std::vector<size_t> bucket_counts; 

    void reset() { *this = SearchStats(); }
};

/**
 * @brief Prints search statistics to an output stream.
 * @param stats The SearchStats object to print.
 * @param out The output stream to write to (defaults to std::cout).
 */
inline void print_stats(const SearchStats& stats, std::ostream& out = std::cout) {
    out << "--- Search Statistics ---" << std::endl;
    
    out << "\n[Macro Metrics]\n";
    out << "Total Search Time: " << stats.total_time_ms << " ms\n";
    if (stats.solution_cost >= 0) {
        out << "Solution Cost: " << stats.solution_cost << "\n";
        out << "Solution Length: " << stats.solution_length << "\n";
    } else {
        out << "No solution found.\n";
    }
    out << "Nodes Expanded: " << stats.nodes_expanded << "\n";
    out << "Nodes Generated: " << stats.nodes_generated << "\n";
    out << "Nodes Reopened: " << stats.nodes_reopened << "\n";

    out << "\n[Queue Micro-Benchmarks (ns)]\n";
    if (stats.count_enqueue > 0) {
      out << "Enqueue      | Count: " << stats.count_enqueue 
          << " | Total: " << stats.time_enqueue << " ns | Avg: " 
          << stats.time_enqueue / stats.count_enqueue << " ns\n";
    }
    if (stats.count_dequeue > 0) {
      out << "Dequeue      | Count: " << stats.count_dequeue 
          << " | Total: " << stats.time_dequeue << " ns | Avg: " 
          << stats.time_dequeue / stats.count_dequeue << " ns\n";
    }
    if (stats.count_change_priority > 0) {
      out << "Change Prio. | Count: " << stats.count_change_priority 
          << " | Total: " << stats.time_change_priority << " ns | Avg: " 
          << stats.time_change_priority / stats.count_change_priority << " ns\n";
    }
    if (stats.count_reorder > 0) {
      out << "Reorder      | Count: " << stats.count_reorder 
          << " | Total: " << stats.time_reorder << " ns | Avg: " 
          << stats.time_reorder / stats.count_reorder << " ns\n";
    }
    
    out << "-------------------------" << std::endl;
}

// C++17 SFINAE helper to detect if a type has a 'rebuild' method.
template<typename, typename, typename = std::void_t<>>
struct has_rebuild : std::false_type {};

template<typename T, typename F>
struct has_rebuild<T, F, std::void_t<decltype(std::declval<T>().rebuild(std::declval<F>()))>> : std::true_type {};


/**
 * @brief A wrapper around a queue to profile its operations.
 * 
 * @tparam QueueType The type of the underlying queue to be profiled.
 */
template <typename QueueType>
class ProfiledQueue {
public:
    ProfiledQueue(QueueType& underlying_queue, SearchStats& stats)
        : queue_(underlying_queue), stats_(stats) {}

    template<typename PriorityType>
    void push(uint32_t handle, PriorityType priority) {
        const auto start = std::chrono::high_resolution_clock::now();
        queue_.push(handle, priority);
        const auto end = std::chrono::high_resolution_clock::now();
        stats_.time_enqueue += std::chrono::duration<double, std::nano>(end - start).count();
        stats_.count_enqueue++;
    }

    uint32_t pop() {
        const auto start = std::chrono::high_resolution_clock::now();
        uint32_t handle = queue_.pop();
        const auto end = std::chrono::high_resolution_clock::now();
        stats_.time_dequeue += std::chrono::duration<double, std::nano>(end - start).count();
        stats_.count_dequeue++;
        return handle;
    }

    template<typename PriorityType>
    void decrease_key(uint32_t handle, PriorityType new_priority) {
        const auto start = std::chrono::high_resolution_clock::now();
        queue_.decrease_key(handle, new_priority);
        const auto end = std::chrono::high_resolution_clock::now();
        stats_.time_change_priority += std::chrono::duration<double, std::nano>(end - start).count();
        stats_.count_change_priority++;
    }

    template<typename Func>
    void rebuild(Func calculate_priority) {
        // Use C++17 SFINAE to conditionally compile this method.
        if constexpr (has_rebuild<QueueType, Func>::value) {
            const auto start = std::chrono::high_resolution_clock::now();
            queue_.rebuild(calculate_priority);
            const auto end = std::chrono::high_resolution_clock::now();
            stats_.time_reorder += std::chrono::duration<double, std::nano>(end - start).count();
            stats_.count_reorder++;
        }
    }

    // Pass-through methods that don't need profiling
    bool empty() const { return queue_.empty(); }
    bool contains(uint32_t handle) const { return queue_.contains(handle); }

private:
    QueueType& queue_;
    SearchStats& stats_;
};

} // namespace utils
