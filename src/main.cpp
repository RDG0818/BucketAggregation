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
#include <type_traits>
#include <sstream>

// Algorithms
#include "algorithms/a_star.h"
#include "algorithms/anytime_a_star.h"
#include "algorithms/ana_star.h"
#include "environments/environments.h"
#include "queues/binary_heap.h"
#include "queues/bucket_queue.h"
#include "queues/bucket_heap.h"
#include "queues/real_bucket_heap.h"
#include "queues/log_bucket_heap.h"
#include "queues/two_level_bucket_queue.h"
#include "utils/utils.h"

// Struct to hold benchmark results
struct BenchmarkResult {
    std::string description;
    utils::SearchStats stats;
    std::string environment_name;
    int instance_id;
};

// Function to write CSV header
void write_csv_header(std::ostream& out) {
    out << "environment,instance,algorithm_description,"
        << "solution_cost,nodes_expanded,nodes_generated,total_time_ms,memory_peak_kb,"
        << "count_enqueue,time_enqueue_ns,count_dequeue,time_dequeue_ns,count_rebuild,time_rebuild_ns,"
        << "count_stale_pops,count_update_pushes,wasted_time_ns,total_overhead_ns\n";
}

// Function to write a result row in CSV format
void write_csv_row(std::ostream& out, const BenchmarkResult& result) {
    const auto& s = result.stats;
    double avg_deq_ns = (s.count_dequeue > 0) ? (s.time_dequeue / s.count_dequeue) : 0;
    double wasted_time_ns = avg_deq_ns * s.count_stale_pops;
    double total_overhead_ns = s.time_enqueue + s.time_dequeue + s.time_rebuild;
    out << result.environment_name << ","
        << result.instance_id << ","
        << "\"" << result.description << "\"" << ","
        << s.solution_cost << ","
        << s.nodes_expanded << ","
        << s.nodes_generated << ","
        << s.total_time_ms << ","
        << s.memory_peak_kb << ","
        << s.count_enqueue << ","
        << s.time_enqueue << ","
        << s.count_dequeue << ","
        << s.time_dequeue << ","
        << s.count_rebuild << ","
        << s.time_rebuild << ","
        << s.count_stale_pops << ","
        << s.count_update_pushes << ","
        << wasted_time_ns << ","
        << total_overhead_ns << "\n";
}


// Function to get terminal width
int get_terminal_width() {
    winsize size;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);
    return size.ws_col;
}

// Functions to print the results table
void print_header(std::ostream& out) {
    out << "\033[1m" << std::left 
              << std::setw(45) << "Algorithm"
              << std::setw(15) << "Sol Cost"
              << std::setw(15) << "Expanded"
              << std::setw(15) << "Generated"
              << std::setw(15) << "Time (ms)"
              << std::setw(15) << "Mem (KB)"
              << "\033[0m" << std::endl;
    out << std::string(get_terminal_width(), '-') << std::endl;
}

void print_row(std::ostream& out, const BenchmarkResult& result) {
    const auto& s = result.stats;
    out << std::left 
              << std::setw(45) << result.description
              << std::setw(15) << s.solution_cost
              << std::setw(15) << s.nodes_expanded
              << std::setw(15) << s.nodes_generated
              << std::setw(15) << std::fixed << std::setprecision(3) << s.total_time_ms
              << std::setw(15) << s.memory_peak_kb
              << std::endl;

    double avg_enq = s.count_enqueue > 0 ? s.time_enqueue / s.count_enqueue : 0;
    double avg_deq = s.count_dequeue > 0 ? s.time_dequeue / s.count_dequeue : 0;
    double avg_reb = s.count_rebuild > 0 ? s.time_rebuild / s.count_rebuild : 0;
    double wasted_time_ms = (avg_deq * s.count_stale_pops) / 1000000.0;

    out << "  \033[2m" // dim color
              << "Queue Ops -> "
              << "Enq: " << s.count_enqueue << " (" << std::fixed << std::setprecision(1) << avg_enq << " ns), "
              << "Deq: " << s.count_dequeue << " (" << std::fixed << std::setprecision(1) << avg_deq << " ns), "
              << "Reb: " << s.count_rebuild << " (" << std::fixed << std::setprecision(1) << avg_reb << " ns), "
              << "Total Overhead: " << std::fixed << std::setprecision(3) << (s.time_enqueue + s.time_dequeue + s.time_rebuild) / 1000000.0 << " ms"
              << "\033[0m" << std::endl;
    out << "  \033[2m"
              << "Waste     -> "
              << "Stale Pops: " << s.count_stale_pops << ", "
              << "Update Pushes: " << s.count_update_pushes << ", "
              << "Wasted Time: " << std::fixed << std::setprecision(3) << wasted_time_ms << " ms"
              << "\033[0m" << std::endl;
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
BenchmarkResult run_benchmark(Environment& env, Queue& queue, const std::string& description, bool show_spinner) {
    utils::SearchStats stats;
    env.reset_search();
    queue.clear();

    using ProfiledQueueType = utils::ProfiledQueue<Queue>;
    ProfiledQueueType profiled_queue(queue, stats);

    using Solver = Algorithm<Environment, ProfiledQueueType>;
    Solver solver(env, profiled_queue, &stats);

    std::atomic<bool> done(false);
    std::thread loading_thread;
    if (show_spinner) {
        loading_thread = std::thread([&]() {
            char spinner[] = "|/-\\";
            int i = 0;
            while (!done) {
                std::cout << "\r" << "Running... " << spinner[i++ % 4] << std::flush;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            std::cout << "\r" << std::string(20, ' ') << "\r"; 
        });
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    solver.solve();
    auto end_time = std::chrono::high_resolution_clock::now();
    
    if (show_spinner) {
        done = true;
        loading_thread.join();
    }

    stats.total_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    stats.memory_peak_kb = utils::get_peak_memory_kb();

    return {description, stats, "", 0};
}

// Function to parse a range string like "0-9" or "5"
std::vector<int> parse_range(const std::string& s) {
    std::vector<int> numbers;
    if (s.empty()) return numbers;
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
void execute_benchmarks(
    Environment& env, 
    const std::vector<std::string>& algorithms_to_run, 
    uint32_t alpha, 
    uint32_t beta, 
    int d_ary,
    std::ostream& out,
    bool is_file_output,
    const std::string& env_name,
    int instance_id) 
{
    auto process_result = [&](BenchmarkResult result) {
        result.environment_name = env_name;
        result.instance_id = instance_id;
        if (is_file_output) {
            write_csv_row(out, result);
        } else {
            print_row(out, result);
        }
    };

    for (const auto& algo_name : algorithms_to_run) {
        if (algo_name == "astar_binary") {
            BinaryHeap<uint32_t> heap;
            process_result(run_benchmark<AStar>(env, heap, "A* with BinaryHeap", !is_file_output));
        } else if (algo_name == "anytime_astar_binary") {
            BinaryHeap<uint32_t> heap;
            process_result(run_benchmark<AnytimeAStar>(env, heap, "Anytime A* with BinaryHeap", !is_file_output));
        } else if (algo_name == "anastar_binary") {
            BinaryHeap<double, std::less<double>> heap;
            process_result(run_benchmark<ANAStar>(env, heap, "ANA* with BinaryHeap", !is_file_output));
        } else if (algo_name == "anastar_bucket") {
            ANAStarPriorityCalculator calculator;
            std::string desc = "ANA* with BucketHeap (D=" + std::to_string(d_ary) + ")";
            switch (d_ary) {
                case 2: {
                    BucketHeap<ANAStarPriorityCalculator, std::less<double>, 2> bucket_heap(calculator);
                    process_result(run_benchmark<ANAStar>(env, bucket_heap, desc, !is_file_output));
                    break;
                }
                case 4: {
                    BucketHeap<ANAStarPriorityCalculator, std::less<double>, 4> bucket_heap(calculator);
                    process_result(run_benchmark<ANAStar>(env, bucket_heap, desc, !is_file_output));
                    break;
                }
                case 8: {
                    BucketHeap<ANAStarPriorityCalculator, std::less<double>, 8> bucket_heap(calculator);
                    process_result(run_benchmark<ANAStar>(env, bucket_heap, desc, !is_file_output));
                    break;
                }
                default:
                    if (!is_file_output) std::cerr << "Unsupported D value for BucketHeap: " << d_ary << std::endl;
            }
        } else if (algo_name == "anastar_logbucket") {
            ANAStarPriorityCalculator calculator;
            std::string desc = "ANA* with LogBucketHeap (D=" + std::to_string(d_ary) + ")";
            switch (d_ary) {
                case 2: {
                    LogBucketHeap<ANAStarPriorityCalculator, std::less<double>, 2> log_bucket_heap(calculator);
                    process_result(run_benchmark<ANAStar>(env, log_bucket_heap, desc, !is_file_output));
                    break;
                }
                case 4: {
                    LogBucketHeap<ANAStarPriorityCalculator, std::less<double>, 4> log_bucket_heap(calculator);
                    process_result(run_benchmark<ANAStar>(env, log_bucket_heap, desc, !is_file_output));
                    break;
                }
                case 8: {
                    LogBucketHeap<ANAStarPriorityCalculator, std::less<double>, 8> log_bucket_heap(calculator);
                    process_result(run_benchmark<ANAStar>(env, log_bucket_heap, desc, !is_file_output));
                    break;
                }
                default:
                    if (!is_file_output) std::cerr << "Unsupported D value for LogBucketHeap: " << d_ary << std::endl;
            }
        } else if (algo_name == "anastar_realbucket") {
            ANAStarPriorityCalculator calculator;
            std::string desc = "ANA* with RealBucketHeap (a=" + std::to_string(alpha) + ", b=" + std::to_string(beta) + ", D=" + std::to_string(d_ary) + ")";
            switch (d_ary) {
                case 2: {
                    RealBucketHeap<ANAStarPriorityCalculator, std::less<double>, 2> real_bucket_heap(calculator, alpha, beta);
                    process_result(run_benchmark<ANAStar>(env, real_bucket_heap, desc, !is_file_output));
                    break;
                }
                case 4: {
                    RealBucketHeap<ANAStarPriorityCalculator, std::less<double>, 4> real_bucket_heap(calculator, alpha, beta);
                    process_result(run_benchmark<ANAStar>(env, real_bucket_heap, desc, !is_file_output));
                    break;
                }
                case 8: {
                    RealBucketHeap<ANAStarPriorityCalculator, std::less<double>, 8> real_bucket_heap(calculator, alpha, beta);
                    process_result(run_benchmark<ANAStar>(env, real_bucket_heap, desc, !is_file_output));
                    break;
                }
                default:
                    if (!is_file_output) std::cerr << "Unsupported D value for RealBucketHeap: " << d_ary << std::endl;
            }
        }
    }
}

template<typename T>
std::vector<T> parse_list(const std::string& s) {
    std::vector<T> result;
    if (s.empty()) return result;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if constexpr (std::is_same_v<T, std::string>) {
            if (!item.empty()) result.push_back(item);
        } else if constexpr (std::is_integral_v<T>) {
            if (!item.empty()) result.push_back(static_cast<T>(std::stoll(item)));
        } else {
            if (!item.empty()) result.push_back(static_cast<T>(std::stod(item)));
        }
    }
    if (result.empty() && !s.empty()) {
        if constexpr (std::is_same_v<T, std::string>) {
            result.push_back(s);
        } else if constexpr (std::is_integral_v<T>) {
            result.push_back(static_cast<T>(std::stoll(s)));
        } else {
            result.push_back(static_cast<T>(std::stod(s)));
        }
    }
    return result;
}


int main(int argc, char** argv) {
    cxxopts::Options options("Benchmark", "A command-line benchmarking tool for search algorithms.");

    options.add_options()
        ("p,pancakes", "Number of pancakes", cxxopts::value<int>()->default_value("48"))
        ("grid-width", "Grid environment width", cxxopts::value<int>()->default_value("1000"))
        ("grid-height", "Grid environment height", cxxopts::value<int>()->default_value("1000"))
        ("grid-max-edge-cost", "Grid environment max edge cost", cxxopts::value<uint32_t>()->default_value("1"))
        ("k,korf", "Korf100 puzzle range (e.g., 0-9 or 5)", cxxopts::value<std::string>()->default_value("0"))
        ("o,output", "Output file for benchmark results (CSV format)", cxxopts::value<std::string>())
        ("num-grid", "Number of random grid instances to run", cxxopts::value<int>()->default_value("1"))
        ("num-pancake", "Number of random pancake instances to run", cxxopts::value<int>()->default_value("1"))
        
        ("grid-alphas", "CSV for Grid alpha", cxxopts::value<std::string>()->default_value("1"))
        ("grid-betas", "CSV for Grid beta", cxxopts::value<std::string>()->default_value("10"))
        ("grid-ds", "CSV for Grid D", cxxopts::value<std::string>()->default_value("2"))

        ("pancake-alphas", "CSV for Pancake alpha", cxxopts::value<std::string>()->default_value("1"))
        ("pancake-betas", "CSV for Pancake beta", cxxopts::value<std::string>()->default_value("2"))
        ("pancake-ds", "CSV for Pancake D", cxxopts::value<std::string>()->default_value("2"))

        ("korf-alphas", "CSV for Korf alpha", cxxopts::value<std::string>()->default_value("1"))
        ("korf-betas", "CSV for Korf beta", cxxopts::value<std::string>()->default_value("10"))
        ("korf-ds", "CSV for Korf D", cxxopts::value<std::string>()->default_value("2"))

        ("e,environments", "Comma-separated list of environments (grid, pancake, korf100, random_grid, heavy_pancake, heavy_korf100)", cxxopts::value<std::string>()->default_value("grid,pancake,korf100"))
        ("l,algorithms", "Comma-separated list of algorithms to run", cxxopts::value<std::string>()->default_value("astar_binary,anytime_astar_binary,anastar_binary,anastar_bucket,anastar_logbucket,anastar_realbucket"))
        ("h,help", "Print usage");

    auto result = options.parse(argc, argv);

    if (result.count("help")) {
        std::cout << options.help() << std::endl;
        return 0;
    }

    std::ofstream outfile;
    bool file_output = result.count("output") > 0;
    std::ostream* out_stream_ptr;

    if (file_output) {
        outfile.open(result["output"].as<std::string>());
        if (!outfile.is_open()) {
            std::cerr << "Error: Could not open output file: " << result["output"].as<std::string>() << std::endl;
            return 1;
        }
        out_stream_ptr = &outfile;
        write_csv_header(*out_stream_ptr);
    } else {
        out_stream_ptr = &std::cout;
        // Clear terminal screen and move cursor to home
        *out_stream_ptr << "\033[2J\033[H";
    }
    std::ostream& out = *out_stream_ptr;

    // Parse algorithm list
    std::vector<std::string> algorithms_to_run;
    std::string algo_str = result["algorithms"].as<std::string>();
    if (algo_str == "all") {
        algorithms_to_run = {"astar_binary", "anytime_astar_binary", "anastar_binary", "anastar_bucket", "anastar_logbucket", "anastar_realbucket"};
    } else {
        algorithms_to_run = parse_list<std::string>(algo_str);
    }
    
    std::vector<std::string> once_off_algos;
    std::vector<std::string> d_sweep_algos;
    std::vector<std::string> full_sweep_algos;
    for(const auto& algo : algorithms_to_run) {
        if (algo == "anastar_realbucket") {
            full_sweep_algos.push_back(algo);
        } else if (algo == "anastar_bucket" || algo == "anastar_logbucket") {
            d_sweep_algos.push_back(algo);
        } else {
            once_off_algos.push_back(algo);
        }
    }

    // Parse environment list
    std::vector<std::string> envs_to_run = parse_list<std::string>(result["environments"].as<std::string>());
    
    int term_width = get_terminal_width();
    int num_grid_runs = result["num-grid"].as<int>();
    int num_pancake_runs = result["num-pancake"].as<int>();

    for (const auto& env_name : envs_to_run) {
        if (env_name == "grid") {
            if (!file_output) {
                out << "\n\033[1m" << "Grid Environment (" 
                        << result["grid-width"].as<int>() << "x" << result["grid-height"].as<int>()
                        << ")" << "\033[0m\n" << std::string(term_width, '=') << "\n";
            }
            
            for (int i = 0; i < num_grid_runs; ++i) {
                if (!file_output) {
                    out << "\033[1;34m" << "Running instance #" << i << "\033[0m" << std::endl;
                    print_header(out);
                }
                GridEnvironment grid_env(result["grid-width"].as<int>(), result["grid-height"].as<int>(), 1 + i);
                
                if (!once_off_algos.empty()) {
                    execute_benchmarks(grid_env, once_off_algos, 0, 0, 0, out, file_output, env_name, i);
                }
                
                if (!d_sweep_algos.empty()) {
                    auto ds = parse_list<int>(result["grid-ds"].as<std::string>());
                    for (int d : ds) {
                        execute_benchmarks(grid_env, d_sweep_algos, 0, 0, d, out, file_output, env_name, i);
                    }
                }

                if (!full_sweep_algos.empty()) {
                    auto alphas = parse_list<uint32_t>(result["grid-alphas"].as<std::string>());
                    auto betas = parse_list<uint32_t>(result["grid-betas"].as<std::string>());
                    auto ds = parse_list<int>(result["grid-ds"].as<std::string>());
                    for (int d : ds) {
                        for (uint32_t alpha : alphas) {
                            for (uint32_t beta : betas) {
                                execute_benchmarks(grid_env, full_sweep_algos, alpha, beta, d, out, file_output, env_name, i);
                            }
                        }
                    }
                }
                if (!file_output) out << std::endl;
            }

        } else if (env_name == "pancake") {
            if (!file_output) {
                out << "\n\033[1m" << "Pancake Puzzle (" << PancakeEnvironment::N << " pancakes)" 
                        << "\033[0m\n" << std::string(term_width, '=') << "\n";
            }
            for (int i = 0; i < num_pancake_runs; ++i) {
                if (!file_output) {
                    out << "\033[1;34m" << "Running instance #" << i << "\033[0m" << std::endl;
                    print_header(out);
                }
                PancakeEnvironment pancake_env(42 + i, 50000000);
                pancake_env.generate_start_node();

                if (!once_off_algos.empty()) {
                    execute_benchmarks(pancake_env, once_off_algos, 0, 0, 0, out, file_output, env_name, i);
                }

                if (!d_sweep_algos.empty()) {
                    auto ds = parse_list<int>(result["pancake-ds"].as<std::string>());
                    for (int d : ds) {
                        execute_benchmarks(pancake_env, d_sweep_algos, 0, 0, d, out, file_output, env_name, i);
                    }
                }
                
                if (!full_sweep_algos.empty()) {
                    auto alphas = parse_list<uint32_t>(result["pancake-alphas"].as<std::string>());
                    auto betas = parse_list<uint32_t>(result["pancake-betas"].as<std::string>());
                    auto ds = parse_list<int>(result["pancake-ds"].as<std::string>());
                    for (int d : ds) {
                        for (uint32_t alpha : alphas) {
                            for (uint32_t beta : betas) {
                                execute_benchmarks(pancake_env, full_sweep_algos, alpha, beta, d, out, file_output, env_name, i);
                            }
                        }
                    }
                }
                if (!file_output) out << std::endl;
            }

        } else if (env_name == "korf100") {
            if (!file_output) {
                out << "\n\033[1m" << "Sliding Tile Puzzle (Korf100)" << "\033[0m\n" << std::string(term_width, '=') << "\n";
            }
            
            auto puzzle_indices = parse_range(result["korf"].as<std::string>());
            for (int index : puzzle_indices) {
                if (!file_output) {
                    out << "\033[1;34m" << "Running puzzle #" << index << "\033[0m" << std::endl;
                    print_header(out);
                }
                try {
                    SlidingTileEnvironment tile_env(index, "korf100.txt", 20000000);

                    if (!once_off_algos.empty()) {
                        execute_benchmarks(tile_env, once_off_algos, 0, 0, 0, out, file_output, env_name, index);
                    }
                    
                    auto ds = parse_list<int>(result["korf-ds"].as<std::string>());
                    if (!d_sweep_algos.empty()) {
                        for (int d : ds) {
                            execute_benchmarks(tile_env, d_sweep_algos, 0, 0, d, out, file_output, env_name, index);
                        }
                    }

                    if (!full_sweep_algos.empty()) {
                        auto alphas = parse_list<uint32_t>(result["korf-alphas"].as<std::string>());
                        auto betas = parse_list<uint32_t>(result["korf-betas"].as<std::string>());
                        for (int d : ds) {
                            for (uint32_t alpha : alphas) {
                                for (uint32_t beta : betas) {
                                    execute_benchmarks(tile_env, full_sweep_algos, alpha, beta, d, out, file_output, env_name, index);
                                }
                            }
                        }
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error setting up SlidingTileEnvironment for puzzle #" << index << ": " << e.what() << std::endl;
                }
                if (!file_output) out << std::endl;
            }
        } else if (env_name == "heavy_pancake") {
            if (!file_output) {
                out << "\n\033[1m" << "Heavy Pancake Puzzle (" << HeavyPancakeEnvironment::N << " pancakes)" 
                        << "\033[0m\n" << std::string(term_width, '=') << "\n";
            }
            for (int i = 0; i < num_pancake_runs; ++i) {
                if (!file_output) {
                    out << "\033[1;34m" << "Running instance #" << i << "\033[0m" << std::endl;
                    print_header(out);
                }
                HeavyPancakeEnvironment heavy_pancake_env(42 + i, 50000000);
                heavy_pancake_env.generate_start_node();

                if (!once_off_algos.empty()) {
                    execute_benchmarks(heavy_pancake_env, once_off_algos, 0, 0, 0, out, file_output, env_name, i);
                }

                if (!d_sweep_algos.empty()) {
                    auto ds = parse_list<int>(result["pancake-ds"].as<std::string>());
                    for (int d : ds) {
                        execute_benchmarks(heavy_pancake_env, d_sweep_algos, 0, 0, d, out, file_output, env_name, i);
                    }
                }
                
                if (!full_sweep_algos.empty()) {
                    auto alphas = parse_list<uint32_t>(result["pancake-alphas"].as<std::string>());
                    auto betas = parse_list<uint32_t>(result["pancake-betas"].as<std::string>());
                    auto ds = parse_list<int>(result["pancake-ds"].as<std::string>());
                    for (int d : ds) {
                        for (uint32_t alpha : alphas) {
                            for (uint32_t beta : betas) {
                                execute_benchmarks(heavy_pancake_env, full_sweep_algos, alpha, beta, d, out, file_output, env_name, i);
                            }
                        }
                    }
                }
                if (!file_output) out << std::endl;
            }

        } else if (env_name == "heavy_korf100") {
            if (!file_output) {
                out << "\n\033[1m" << "Heavy Sliding Tile Puzzle (Korf100)" << "\033[0m\n" << std::string(term_width, '=') << "\n";
            }
            
            auto puzzle_indices = parse_range(result["korf"].as<std::string>());
            for (int index : puzzle_indices) {
                if (!file_output) {
                    out << "\033[1;34m" << "Running puzzle #" << index << "\033[0m" << std::endl;
                    print_header(out);
                }
                try {
                    // Note: Using the new HeavySlidingTileEnvironment class
                    HeavySlidingTileEnvironment heavy_tile_env(index, "korf100.txt", 20000000);

                    if (!once_off_algos.empty()) {
                        execute_benchmarks(heavy_tile_env, once_off_algos, 0, 0, 0, out, file_output, env_name, index);
                    }
                    
                    auto ds = parse_list<int>(result["korf-ds"].as<std::string>());
                    if (!d_sweep_algos.empty()) {
                        for (int d : ds) {
                            execute_benchmarks(heavy_tile_env, d_sweep_algos, 0, 0, d, out, file_output, env_name, index);
                        }
                    }

                    if (!full_sweep_algos.empty()) {
                        auto alphas = parse_list<uint32_t>(result["korf-alphas"].as<std::string>());
                        auto betas = parse_list<uint32_t>(result["korf-betas"].as<std::string>());
                        for (int d : ds) {
                            for (uint32_t alpha : alphas) {
                                for (uint32_t beta : betas) {
                                    execute_benchmarks(heavy_tile_env, full_sweep_algos, alpha, beta, d, out, file_output, env_name, index);
                                }
                            }
                        }
                    }
                } catch (const std::exception& e) {
                    std::cerr << "Error setting up HeavySlidingTileEnvironment for puzzle #" << index << ": " << e.what() << std::endl;
                }
                if (!file_output) out << std::endl;
            }
        } else if (env_name == "random_grid") {
            if (!file_output) {
                out << "\n\033[1m" << "Random Grid Environment (" 
                        << result["grid-width"].as<int>() << "x" << result["grid-height"].as<int>()
                        << ")" << "\033[0m\n" << std::string(term_width, '=') << "\n";
            }

            for (int i = 0; i < num_grid_runs; ++i) {
                if (!file_output) {
                    out << "\033[1;34m" << "Running instance #" << i << "\033[0m" << std::endl;
                    print_header(out);
                }
                RandomGridEnvironment rand_grid_env(result["grid-width"].as<int>(), result["grid-height"].as<int>(), result["grid-max-edge-cost"].as<uint32_t>(), 42 + i);
                
                if (!once_off_algos.empty()) {
                    execute_benchmarks(rand_grid_env, once_off_algos, 0, 0, 0, out, file_output, env_name, i);
                }
                
                if (!d_sweep_algos.empty()) {
                    auto ds = parse_list<int>(result["grid-ds"].as<std::string>());
                    for (int d : ds) {
                        execute_benchmarks(rand_grid_env, d_sweep_algos, 0, 0, d, out, file_output, env_name, i);
                    }
                }

                if (!full_sweep_algos.empty()) {
                    auto alphas = parse_list<uint32_t>(result["grid-alphas"].as<std::string>());
                    auto betas = parse_list<uint32_t>(result["grid-betas"].as<std::string>());
                    auto ds = parse_list<int>(result["grid-ds"].as<std::string>());
                    for (int d : ds) {
                        for (uint32_t alpha : alphas) {
                            for (uint32_t beta : betas) {
                                execute_benchmarks(rand_grid_env, full_sweep_algos, alpha, beta, d, out, file_output, env_name, i);
                            }
                        }
                    }
                }
                if (!file_output) out << std::endl;
            }
        } 
    }

    if (file_output) {
        outfile.close();
        std::cout << "Benchmark results written to " << result["output"].as<std::string>() << std::endl;
    }

    return 0;
}
