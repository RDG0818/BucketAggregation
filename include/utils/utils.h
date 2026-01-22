#pragma once

#include <chrono>
#include <functional>
#include <iostream>
#include <string>
#include <sstream>
#include <utility>

#ifdef __linux__
#include <fstream>
#include <unistd.h>
#endif

namespace utils {

/**
 * @brief Measures the execution time of a callable.
 * 
 * @tparam TimeT The std::chrono::duration type to return (e.g., std::chrono::microseconds).
 * @tparam F The type of the callable.
 * @tparam Args The types of the arguments to the callable.
 * @param func The callable to execute and measure.
 * @param args The arguments to pass to the callable.
 * @return The execution time as a std::chrono::duration.
 */
template<typename TimeT = std::chrono::microseconds, typename F, typename... Args>
TimeT measure_time(F&& func, Args&&... args) {
    auto start = std::chrono::high_resolution_clock::now();
    std::forward<F>(func)(std::forward<Args>(args)...);
    auto end = std::chrono::high_resolution_clock::now();
    return std::chrono::duration_cast<TimeT>(end - start);
}

/**
 * @brief Measures the execution time of a callable and returns its result.
 *
 * @tparam TimeT The std::chrono::duration type for the time measurement.
 * @tparam F The type of the callable.
 * @tparam Args The types of the arguments to the callable.
 * @param func The callable to execute and measure.
 * @param args The arguments to pass to the callable.
 * @return A std::pair containing the result of the callable and the execution time.
 */
template<typename TimeT = std::chrono::microseconds, typename F, typename... Args>
auto measure_time_with_result(F&& func, Args&&... args)
    -> std::pair<decltype(std::forward<F>(func)(std::forward<Args>(args)...)), TimeT> {
    auto start = std::chrono::high_resolution_clock::now();
    auto result = std::forward<F>(func)(std::forward<Args>(args)...);
    auto end = std::chrono::high_resolution_clock::now();
    return {result, std::chrono::duration_cast<TimeT>(end - start)};
}


/**
 * @brief Gets the current resident set size (RSS) memory usage in bytes.
 * 
 * This implementation is for Linux only. On other operating systems, it will return 0.
 * It works by reading the '/proc/self/statm' file.
 * 
 * @return The current RSS memory usage in bytes, or 0 if not on Linux or on error.
 */
inline long get_memory_usage_bytes() {
#ifdef __linux__
    std::ifstream statm("/proc/self/statm");
    if (!statm) {
        return 0;
    }
    long size, resident, share, text, lib, data, dt;
    statm >> size >> resident >> share >> text >> lib >> data >> dt;
    return resident * sysconf(_SC_PAGESIZE);
#else
    return 0; // Not implemented for non-Linux systems
#endif
}

// A struct to hold common statistics from a search algorithm's execution.
struct SearchStatistics {
    std::chrono::microseconds execution_time{0};
    long peak_memory_bytes{0};
    size_t nodes_expanded{0};
    double solution_cost{-1.0};
    size_t solution_length{0};
};

/**
 * @brief Prints search statistics to an output stream.
 * 
 * @param stats The SearchStatistics object to print.
 * @param out The output stream to write to (defaults to std::cout).
 */
inline void print_stats(const SearchStatistics& stats, std::ostream& out = std::cout) {
    out << "--- Search Statistics ---\n";
    out << "Execution Time: " << stats.execution_time.count() << " us\n";
    if (stats.peak_memory_bytes > 0) {
        out << "Peak Memory: " << stats.peak_memory_bytes / 1024 << " KB\n";
    }
    if (stats.nodes_expanded > 0) {
        out << "Nodes Expanded: " << stats.nodes_expanded << "\n";
    }
    if (stats.solution_cost >= 0) {
        out << "Solution Cost: " << stats.solution_cost << "\n";
        out << "Solution Length: " << stats.solution_length << "\n";
    } else {
        out << "No solution found.\n";
    }
    out << "-------------------------\n";
}

} // namespace utils