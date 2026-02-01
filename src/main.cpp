// src/main.cpp

#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <fstream>
#include <iomanip>
#include <map>
#include <algorithm>
#include <cxxopts.hpp>
#include <sys/ioctl.h>
#include <unistd.h>
#include <thread>
#include <atomic>
#include <chrono>

// Algorithms
#include "algorithms/a_star.h"
#include "algorithms/anytime_a_star.h"
#include "algorithms/ana_star.h"
#include "environments/environments.h"
#include "queues/binary_heap.h"
#include "queues/bucket_queue.h"
#include "queues/indexed_binary_heap.h"
#include "queues/bucket_heap.h"
#include "queues/real_bucket_heap.h"
#include "queues/two_level_bucket_queue.h"
#include "utils/utils.h"

// Struct to hold benchmark results
struct BenchmarkResult {
    std::string description;
    utils::SearchStats stats;
};

// Function to get terminal width
int get_terminal_width() {
    winsize size;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);
    return size.ws_col;
}

// Functions to print the results table
void print_header() {
    std::cout << "\033[1m" << std::left << std::setw(45) << "Algorithm"
              << std::setw(20) << "Solution Cost"
              << std::setw(20) << "Nodes Expanded"
              << std::setw(20) << "Nodes Generated"
              << std::setw(20) << "Time (ms)"
              << std::setw(20) << "Memory (KB)" << "\033[0m" << std::endl;
    std::cout << std::string(get_terminal_width(), '-') << std::endl;
}

void print_row(const BenchmarkResult& result) {
    std::cout << std::left << std::setw(45) << result.description
              << std::setw(20) << result.stats.solution_cost
              << std::setw(20) << result.stats.nodes_expanded
              << std::setw(20) << result.stats.nodes_generated
              << std::setw(20) << std::fixed << std::setprecision(3) << result.stats.total_time_ms
              << std::setw(20) << result.stats.memory_peak_kb << std::endl;
}


// ANA* Priority Calculator
struct ANAStarPriorityCalculator {
    double G_upper = std::numeric_limits<double>::max();
    void set_g_upper(double g) { G_upper = g; }
    double operator()(double f, double h) const {
        if (h == 0) return std::numeric_limits<double>::max();
        double g = f - h;
        if (g >= G_upper) return std::numeric_limits<double>::lowest();
        return (G_upper - g) / h;
    }
};

// Main benchmark runner
template <template<typename, typename> class Algorithm, typename Environment, typename Queue>
void run_benchmark(Environment& env, Queue& queue, const std::string& description) {
    utils::SearchStats stats;
    env.reset_search();
    queue.clear();

    using ProfiledQueueType = utils::ProfiledQueue<Queue>;
    ProfiledQueueType profiled_queue(queue, stats);

    using Solver = Algorithm<Environment, ProfiledQueueType>;
    Solver solver(env, profiled_queue, &stats);

    std::atomic<bool> done(false);
    std::thread loading_thread([&]() {
        char spinner[] = "|/-\\";
        int i = 0;
        while (!done) {
            std::cout << "\r" << "Running... " << spinner[i++ % 4] << std::flush;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::cout << "\r" << std::string(20, ' ') << "\r"; 
    });

    auto start_time = std::chrono::high_resolution_clock::now();
    solver.solve();
    auto end_time = std::chrono::high_resolution_clock::now();
    
    done = true;
    loading_thread.join();

    stats.total_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    stats.memory_peak_kb = utils::get_peak_memory_kb();

    print_row({description, stats});
}

// Function to parse a range string like "0-9" or "5"
std::vector<int> parse_range(const std::string& s) {
    std::vector<int> numbers;
    size_t dash_pos = s.find('-');
    if (dash_pos != std::string::npos) {
        int start = std::stoi(s.substr(0, dash_pos));
        int end = std::stoi(s.substr(dash_pos + 1));
        for (int i = start; i <= end; ++i) {
            numbers.push_back(i);
        }
    } else {
        numbers.push_back(std::stoi(s));
    }
    return numbers;
}

template<typename Environment>
void execute_benchmarks(Environment& env, const std::vector<std::string>& algorithms_to_run, uint32_t alpha, uint32_t beta) {
    print_header();
    for (const auto& algo_name : algorithms_to_run) {
        if (algo_name == "astar_binary") {
            BinaryHeap<uint32_t> heap;
            run_benchmark<AStar>(env, heap, "A* with BinaryHeap");
        } else if (algo_name == "anytime_astar_binary") {
            BinaryHeap<uint32_t> heap;
            run_benchmark<AnytimeAStar>(env, heap, "Anytime A* with BinaryHeap");
        } else if (algo_name == "anastar_binary") {
            BinaryHeap<double, std::less<double>> heap;
            run_benchmark<ANAStar>(env, heap, "ANA* with BinaryHeap");
        } else if (algo_name == "anastar_bucket") {
            ANAStarPriorityCalculator calculator;
            BucketHeap<ANAStarPriorityCalculator, std::less<double>> bucket_heap(calculator);
            run_benchmark<ANAStar>(env, bucket_heap, "ANA* with BucketHeap");
        } else if (algo_name == "anastar_realbucket") {
            ANAStarPriorityCalculator calculator;
            RealBucketHeap<ANAStarPriorityCalculator, std::less<double>> real_bucket_heap(calculator, alpha, beta);
            std::string desc = "ANA* with RealBucketHeap (a=" + std::to_string(alpha) + ", b=" + std::to_string(beta) + ")";
            run_benchmark<ANAStar>(env, real_bucket_heap, desc);
        }
    }
    std::cout << std::endl;
}


int main(int argc, char** argv) {
    // Clear terminal screen and move cursor to home
    std::cout << "\033[2J\033[H";

    cxxopts::Options options("Benchmark", "A command-line benchmarking tool for search algorithms.");

    options.add_options()
        ("p,pancakes", "Number of pancakes", cxxopts::value<int>()->default_value("42"))
        ("grid-width", "Grid environment width", cxxopts::value<int>()->default_value("1000"))
        ("grid-height", "Grid environment height", cxxopts::value<int>()->default_value("1000"))
        ("k,korf", "Korf100 puzzle range (e.g., 0-9 or 5)", cxxopts::value<std::string>()->default_value("0"))
        ("a,alpha", "Default alpha for RealBucketHeap", cxxopts::value<uint32_t>()->default_value("1"))
        ("b,beta", "Default beta for RealBucketHeap", cxxopts::value<uint32_t>()->default_value("10"))
        ("grid-alpha", "Alpha for Grid env", cxxopts::value<uint32_t>())
        ("grid-beta", "Beta for Grid env", cxxopts::value<uint32_t>())
        ("pancake-alpha", "Alpha for Pancake env", cxxopts::value<uint32_t>())
        ("pancake-beta", "Beta for Pancake env", cxxopts::value<uint32_t>())
        ("korf-alpha", "Alpha for Korf100 env", cxxopts::value<uint32_t>())
        ("korf-beta", "Beta for Korf100 env", cxxopts::value<uint32_t>())
        ("e,environments", "Comma-separated list of environments (grid, pancake, korf100)", cxxopts::value<std::string>()->default_value("grid,pancake,korf100"))
        ("l,algorithms", "Comma-separated list of algorithms to run", cxxopts::value<std::string>()->default_value("astar_binary,anytime_astar_binary,anastar_binary,anastar_bucket,anastar_realbucket"))
        ("h,help", "Print usage");

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    uint32_t default_alpha = result["alpha"].as<uint32_t>();
    uint32_t default_beta = result["beta"].as<uint32_t>();

    uint32_t grid_alpha = result.count("grid-alpha") ? result["grid-alpha"].as<uint32_t>() : default_alpha;
    uint32_t grid_beta = result.count("grid-beta") ? result["grid-beta"].as<uint32_t>() : default_beta;

    uint32_t pancake_alpha = result.count("pancake-alpha") ? result["pancake-alpha"].as<uint32_t>() : default_alpha;
    uint32_t pancake_beta = result.count("pancake-beta") ? result["pancake-beta"].as<uint32_t>() : default_beta;

    uint32_t korf_alpha = result.count("korf-alpha") ? result["korf-alpha"].as<uint32_t>() : default_alpha;
    uint32_t korf_beta = result.count("korf-beta") ? result["korf-beta"].as<uint32_t>() : default_beta;

    // Parse algorithm list
    std::vector<std::string> algorithms_to_run;
    std::string algo_str = result["algorithms"].as<std::string>();
    if (algo_str == "all") {
        algorithms_to_run = {"astar_binary", "anytime_astar_binary", "anastar_binary", "anastar_bucket", "anastar_realbucket"};
    } else {
        std::stringstream ss(algo_str);
        std::string item;
        while (std::getline(ss, item, ',')) {
            algorithms_to_run.push_back(item);
        }
    }

    // Parse environment list
    std::string env_str = result["environments"].as<std::string>();
    std::stringstream ss(env_str);
    std::string item;
    std::vector<std::string> envs_to_run;
    while (std::getline(ss, item, ',')) {
        envs_to_run.push_back(item);
    }
    
    int term_width = get_terminal_width();

    for (const auto& env_name : envs_to_run) {
        if (env_name == "grid") {
            std::cout << "\n\033[1m" << "Grid Environment (" 
                      << result["grid-width"].as<int>() << "x" << result["grid-height"].as<int>()
                      << ")" << "\033[0m\n" << std::string(term_width, '=') << "\n";
            GridEnvironment grid_env(result["grid-width"].as<int>(), result["grid-height"].as<int>(), 42);
            execute_benchmarks(grid_env, algorithms_to_run, grid_alpha, grid_beta);
        } else if (env_name == "pancake") {
            std::cout << "\n\033[1m" << "Pancake Puzzle (" << result["pancakes"].as<int>() << " pancakes)" 
                      << "\033[0m\n" << std::string(term_width, '=') << "\n";
            PancakeEnvironment pancake_env(result["pancakes"].as<int>(), 50000000);
            pancake_env.generate_start_node();
            execute_benchmarks(pancake_env, algorithms_to_run, pancake_alpha, pancake_beta);
        } else if (env_name == "korf100") {
            std::cout << "\n\033[1m" << "Sliding Tile Puzzle (Korf100)" << "\033[0m\n" << std::string(term_width, '=') << "\n";
            auto puzzle_indices = parse_range(result["korf"].as<std::string>());
            for (int index : puzzle_indices) {
                std::cout << "\033[1;34m" << "Running puzzle #" << index << "\033[0m" << std::endl;
                try {
                    SlidingTileEnvironment tile_env(index, "korf100.txt", 20000000);
                    execute_benchmarks(tile_env, algorithms_to_run, korf_alpha, korf_beta);
                } catch (const std::exception& e) {
                    std::cerr << "Error setting up SlidingTileEnvironment for puzzle #" << index << ": " << e.what() << std::endl;
                }
            }
        }
    }

    return 0;
}
