// src/main.cpp

#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <sstream>
#include <thread>
#include <type_traits>
#include <cxxopts.hpp>
#include <sys/ioctl.h>
#include <unistd.h>
#ifdef __linux__
#include <linux/perf_event.h>
#include <sys/syscall.h>
#endif

#include "algorithms/a_star.h"
#include "algorithms/anytime_a_star.h"
#include "algorithms/ana_star.h"
#include "algorithms/dps.h"
#include "environments/environments.h"
#include "queues/bucket_queue.h"
#include "queues/bucket_heap.h"
#include "queues/two_level_bucket_queue.h"
#include "utils/utils.h"

#ifdef __linux__
// RAII wrapper around perf_event_open file descriptors.
// Opens one fd per counter; reset+enable before solve(), disable+read after.
// Degrades gracefully if perf_event_open is unavailable (available=false).
struct PerfCounters {
    int fd_llc_misses    = -1;
    int fd_llc_refs      = -1;
    int fd_branch_misses = -1;
    int fd_branches      = -1;
    int fd_cycles        = -1;
    int fd_instructions  = -1;
    // Software events — available regardless of perf_event_paranoid level.
    int fd_faults_minor  = -1;
    int fd_faults_major  = -1;
    bool available       = false;

    explicit PerfCounters() {
        auto open_hw = [](uint64_t config) -> int {
            perf_event_attr attr{};
            attr.type           = PERF_TYPE_HARDWARE;
            attr.size           = sizeof(attr);
            attr.config         = config;
            attr.disabled       = 1;
            attr.exclude_kernel = 1;
            attr.exclude_hv     = 1;
            return static_cast<int>(syscall(SYS_perf_event_open, &attr, 0, -1, -1, 0));
        };
        auto open_cache = [](uint64_t cache_id, uint64_t op, uint64_t result) -> int {
            perf_event_attr attr{};
            attr.type           = PERF_TYPE_HW_CACHE;
            attr.size           = sizeof(attr);
            attr.config         = cache_id | (op << 8) | (result << 16);
            attr.disabled       = 1;
            attr.exclude_kernel = 1;
            attr.exclude_hv     = 1;
            return static_cast<int>(syscall(SYS_perf_event_open, &attr, 0, -1, -1, 0));
        };
        auto open_sw = [](uint64_t config) -> int {
            perf_event_attr attr{};
            attr.type     = PERF_TYPE_SOFTWARE;
            attr.size     = sizeof(attr);
            attr.config   = config;
            attr.disabled = 1;
            return static_cast<int>(syscall(SYS_perf_event_open, &attr, 0, -1, -1, 0));
        };

        fd_llc_misses    = open_cache(PERF_COUNT_HW_CACHE_LL, PERF_COUNT_HW_CACHE_OP_READ, PERF_COUNT_HW_CACHE_RESULT_MISS);
        fd_llc_refs      = open_cache(PERF_COUNT_HW_CACHE_LL, PERF_COUNT_HW_CACHE_OP_READ, PERF_COUNT_HW_CACHE_RESULT_ACCESS);
        fd_branch_misses = open_hw(PERF_COUNT_HW_BRANCH_MISSES);
        fd_branches      = open_hw(PERF_COUNT_HW_BRANCH_INSTRUCTIONS);
        fd_cycles        = open_hw(PERF_COUNT_HW_CPU_CYCLES);
        fd_instructions  = open_hw(PERF_COUNT_HW_INSTRUCTIONS);
        fd_faults_minor  = open_sw(PERF_COUNT_SW_PAGE_FAULTS_MIN);
        fd_faults_major  = open_sw(PERF_COUNT_SW_PAGE_FAULTS_MAJ);

        available = (fd_llc_misses != -1 || fd_branch_misses != -1 || fd_faults_minor != -1);
    }

    ~PerfCounters() {
        for (int fd : {fd_llc_misses, fd_llc_refs, fd_branch_misses, fd_branches,
                       fd_cycles, fd_instructions, fd_faults_minor, fd_faults_major})
            if (fd != -1) close(fd);
    }

    void reset_and_enable() const {
        for (int fd : {fd_llc_misses, fd_llc_refs, fd_branch_misses, fd_branches,
                       fd_cycles, fd_instructions, fd_faults_minor, fd_faults_major}) {
            if (fd != -1) {
                ioctl(fd, PERF_EVENT_IOC_RESET,  0);
                ioctl(fd, PERF_EVENT_IOC_ENABLE, 0);
            }
        }
    }

    void disable_and_read(utils::SearchStats& s) const {
        for (int fd : {fd_llc_misses, fd_llc_refs, fd_branch_misses, fd_branches,
                       fd_cycles, fd_instructions, fd_faults_minor, fd_faults_major})
            if (fd != -1) ioctl(fd, PERF_EVENT_IOC_DISABLE, 0);

        auto rd = [](int fd) -> uint64_t {
            if (fd == -1) return 0;
            uint64_t val = 0;
            ::read(fd, &val, sizeof(val));
            return val;
        };

        s.perf_llc_misses    = rd(fd_llc_misses);
        s.perf_llc_refs      = rd(fd_llc_refs);
        s.perf_branch_misses = rd(fd_branch_misses);
        s.perf_branches      = rd(fd_branches);
        s.perf_cycles        = rd(fd_cycles);
        s.perf_instructions  = rd(fd_instructions);
        s.perf_faults_minor  = rd(fd_faults_minor);
        s.perf_faults_major  = rd(fd_faults_major);
    }
};
#endif // __linux__

// ─── Result ───────────────────────────────────────────────────────────────────

struct BenchmarkResult {
    std::string description;
    utils::SearchStats stats;
    std::string environment_name;
    int instance_id;
};

// ─── CSV Output ───────────────────────────────────────────────────────────────

void write_csv_header(std::ostream& out) {
    out << "environment,instance,algorithm_description,"
        << "solution_cost,nodes_expanded,nodes_generated,total_time_ms,memory_peak_kb,"
        << "count_enqueue,time_enqueue_ns,count_dequeue,time_dequeue_ns,count_rebuild,time_rebuild_ns,"
        << "count_decrease_key,time_decrease_key_ns,"
        << "count_stale_pops,count_update_pushes,wasted_time_ns,total_overhead_ns,"
        << "total_hmin_scans,total_secondary_bucket_allocs,"
        << "perf_llc_misses,perf_llc_refs,perf_branch_misses,perf_branches,"
        << "perf_cycles,perf_instructions,perf_faults_minor,perf_faults_major\n";
}

void write_csv_row(std::ostream& out, const BenchmarkResult& r) {
    const auto& s = r.stats;
    double avg_deq_ns      = s.count_dequeue > 0 ? s.time_dequeue / s.count_dequeue : 0;
    double wasted_time_ns  = avg_deq_ns * s.count_stale_pops;
    double total_overhead  = s.time_enqueue + s.time_dequeue + s.time_rebuild + s.time_decrease_key;
    out << r.environment_name << ","
        << r.instance_id << ","
        << "\"" << r.description << "\","
        << s.solution_cost << "," << s.nodes_expanded << "," << s.nodes_generated << ","
        << s.total_time_ms << "," << s.memory_peak_kb << ","
        << s.count_enqueue << "," << s.time_enqueue << ","
        << s.count_dequeue << "," << s.time_dequeue << ","
        << s.count_rebuild << "," << s.time_rebuild << ","
        << s.count_decrease_key << "," << s.time_decrease_key << ","
        << s.count_stale_pops << "," << s.count_update_pushes << ","
        << wasted_time_ns << "," << total_overhead << ","
        << s.total_hmin_scans << "," << s.total_secondary_bucket_allocs << ","
        << s.perf_llc_misses << "," << s.perf_llc_refs << ","
        << s.perf_branch_misses << "," << s.perf_branches << ","
        << s.perf_cycles << "," << s.perf_instructions << ","
        << s.perf_faults_minor << "," << s.perf_faults_major << "\n";
}

// ─── Table Output ─────────────────────────────────────────────────────────────

// ANSI helpers
#define BOLD    "\033[1m"
#define DIM     "\033[2m"
#define CYAN    "\033[36m"
#define YELLOW  "\033[33m"
#define GREEN   "\033[32m"
#define RESET   "\033[0m"

int get_terminal_width() {
    winsize size;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);
    return size.ws_col > 0 ? size.ws_col : 100;
}

// Formats a nanosecond average as "123ns" or "1.2us" or "-"
static std::string fmt_ns(double ns) {
    if (ns <= 0) return "-";
    std::ostringstream ss;
    if (ns < 1000) ss << std::fixed << std::setprecision(0) << ns << "ns";
    else            ss << std::fixed << std::setprecision(1) << ns / 1000.0 << "us";
    return ss.str();
}

// Formats a millisecond duration as "1.234"
static std::string fmt_ms(double ms) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(3) << ms;
    return ss.str();
}

// Formats a hardware counter as "1.2B", "456.7M", "12.3K", or the raw number
static std::string fmt_count(uint64_t n) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1);
    if      (n >= 1'000'000'000) ss << n / 1e9 << "B";
    else if (n >= 1'000'000)     ss << n / 1e6 << "M";
    else if (n >= 1'000)         ss << n / 1e3 << "K";
    else                         ss << n;
    return ss.str();
}

// Formats a large integer with comma separators: 1234567 -> "1,234,567"
static std::string fmt_int(uint64_t n) {
    std::string s = std::to_string(n);
    for (int i = static_cast<int>(s.size()) - 3; i > 0; i -= 3)
        s.insert(i, ",");
    return s;
}

void print_env_header(std::ostream& out, const std::string& title, const std::string& subtitle = "") {
    int w = get_terminal_width();
    out << "\n" BOLD CYAN "  " << title << RESET;
    if (!subtitle.empty()) out << DIM "  --  " << subtitle << RESET;
    out << "\n" DIM "  " << std::string(w - 2, '-') << RESET "\n\n";
}

void print_instance_header(std::ostream& out, int instance_id) {
    out << BOLD "  Instance #" << instance_id << RESET "\n";
}

void print_header(std::ostream& out) {
    out << DIM
        << "  " << std::left  << std::setw(48) << "Algorithm"
        << std::right
        << std::setw(9)  << "Cost"
        << std::setw(13) << "Expanded"
        << std::setw(12) << "Time (ms)"
        << std::setw(10) << "Mem (KB)"
        << RESET "\n"
        << DIM "  " << std::string(get_terminal_width() - 2, '-') << RESET "\n";
}

void print_row(std::ostream& out, const BenchmarkResult& r) {
    const auto& s = r.stats;

    double avg_enq   = s.count_enqueue     > 0 ? s.time_enqueue     / s.count_enqueue     : 0;
    double avg_deq   = s.count_dequeue     > 0 ? s.time_dequeue     / s.count_dequeue     : 0;
    double avg_reb   = s.count_rebuild     > 0 ? s.time_rebuild     / s.count_rebuild     : 0;
    double avg_dk    = s.count_decrease_key > 0 ? s.time_decrease_key / s.count_decrease_key : 0;
    double overhead  = (s.time_enqueue + s.time_dequeue + s.time_rebuild + s.time_decrease_key) / 1e6;
    double wasted_ms = (avg_deq * s.count_stale_pops) / 1e6;

    // Main result line
    out << BOLD "  " << std::left << std::setw(48) << r.description << RESET
        << std::right
        << YELLOW << std::setw(9)  << s.solution_cost << RESET
        << GREEN  << std::setw(13) << fmt_int(s.nodes_expanded) << RESET
        << std::setw(12) << fmt_ms(s.total_time_ms)
        << std::setw(10) << s.memory_peak_kb
        << "\n";

    // Queue timing (dim)
    out << DIM "    "
        << "enq "  << fmt_int(s.count_enqueue)      << " @ " << fmt_ns(avg_enq) << "  "
        << "deq "  << fmt_int(s.count_dequeue)      << " @ " << fmt_ns(avg_deq) << "  "
        << "dkey " << fmt_int(s.count_decrease_key) << " @ " << fmt_ns(avg_dk)  << "  "
        << "reb "  << fmt_int(s.count_rebuild)      << " @ " << fmt_ns(avg_reb) << "  "
        << "|  overhead " << fmt_ms(overhead) << " ms"
        << RESET "\n";

    // Stale / internal (dim, only printed when non-zero)
    bool has_waste    = s.count_stale_pops > 0 || s.count_update_pushes > 0;
    bool has_internal = s.total_hmin_scans > 0 || s.total_secondary_bucket_allocs > 0;
    if (has_waste || has_internal) {
        out << DIM "    ";
        if (has_waste) {
            out << "stale " << fmt_int(s.count_stale_pops) << "  "
                << "repush " << fmt_int(s.count_update_pushes) << "  "
                << "wasted " << fmt_ms(wasted_ms) << " ms";
        }
        if (has_waste && has_internal) out << "  |  ";
        if (has_internal) {
            out << "hmin-scans " << fmt_int(s.total_hmin_scans) << "  "
                << "bkt-alloc " << fmt_int(s.total_secondary_bucket_allocs);
        }
        out << RESET "\n";
    }

    // Hardware/software perf counters (dim, only printed when available)
    bool has_perf = s.perf_llc_refs > 0 || s.perf_branch_misses > 0
                 || s.perf_cycles > 0    || s.perf_faults_minor > 0 || s.perf_faults_major > 0;
    if (has_perf) {
        std::ostringstream perf_line;
        bool need_sep = false;
        auto sep = [&] { if (need_sep) perf_line << "  |  "; need_sep = true; };

        perf_line << "    ";
        if (s.perf_llc_refs > 0) {
            std::ostringstream pct;
            pct << std::fixed << std::setprecision(1)
                << (100.0 * s.perf_llc_misses / s.perf_llc_refs) << "%";
            sep();
            perf_line << "LLC  "
                      << fmt_count(s.perf_llc_misses) << " miss / "
                      << fmt_count(s.perf_llc_refs)   << " ref ("
                      << pct.str() << ")";
        }
        if (s.perf_branches > 0) {
            std::ostringstream pct;
            pct << std::fixed << std::setprecision(1)
                << (100.0 * s.perf_branch_misses / s.perf_branches) << "%";
            sep();
            perf_line << "br-miss  "
                      << fmt_count(s.perf_branch_misses) << " / "
                      << fmt_count(s.perf_branches)      << " ("
                      << pct.str() << ")";
        }
        if (s.perf_faults_minor > 0 || s.perf_faults_major > 0) {
            sep();
            perf_line << "pg-flt  "
                      << fmt_count(s.perf_faults_minor) << " min  "
                      << fmt_count(s.perf_faults_major) << " maj";
        }
        if (s.perf_cycles > 0 && s.perf_instructions > 0) {
            std::ostringstream ipc;
            ipc << std::fixed << std::setprecision(2)
                << (static_cast<double>(s.perf_instructions) / s.perf_cycles);
            sep();
            perf_line << "IPC " << ipc.str();
        }
        out << DIM << perf_line.str() << RESET "\n";
    }

    out << "\n";
}

// ─── Priority Calculators ─────────────────────────────────────────────────────

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

struct DPSPriorityCalculator {
    double f_min = 0;
    double epsilon = 1.5;
    void set_f_min(double f) { f_min = f; }
    void set_epsilon(double e) { epsilon = e; }
    double operator()(double f, double h) const {
        if (h == 0) return std::numeric_limits<double>::max();
        double g = f - h;
        double bound = epsilon * f_min;
        if (g >= bound) return std::numeric_limits<double>::lowest();
        return (bound - g) / h;
    }
};

// ─── Benchmark Runner ─────────────────────────────────────────────────────────

template <template<typename, typename> class Algorithm, typename Environment, typename Queue>
BenchmarkResult run_benchmark(Environment& env, Queue& queue, const std::string& description,
                               bool show_spinner, bool collect_metrics, double weight = 1.5) {
    utils::SearchStats stats;
    env.reset_search();
    queue.clear();

    utils::ProfiledQueue<Queue> profiled(queue, stats);
    Algorithm<Environment, utils::ProfiledQueue<Queue>> solver(env, profiled, &stats, collect_metrics, weight);

    std::atomic<bool> done(false);
    std::thread spinner_thread;
    auto t0 = std::chrono::steady_clock::now();

    if (show_spinner) {
        // Truncate the description to fit cleanly on one line.
        std::string short_desc = description.size() > 44
            ? description.substr(0, 41) + "..."
            : description;

        spinner_thread = std::thread([&, short_desc]() {
            const char* frames[] = {"|", "/", "-", "\\"};
            int frame = 0;
            int w = get_terminal_width();

            while (!done) {
                auto elapsed = std::chrono::duration<double>(
                    std::chrono::steady_clock::now() - t0).count();

                // Intentional unsynchronized read — display only, stale value is fine.
                uint64_t expanded = stats.nodes_expanded;

                std::ostringstream line;
                line << "  [" << frames[frame++ % 4] << "]  "
                     << std::left  << std::setw(46) << short_desc
                     << std::right << std::setw(14) << fmt_int(expanded) << " nodes  "
                     << std::fixed << std::setprecision(1) << elapsed << "s";

                std::string s = "\r" + line.str();
                // Pad to overwrite any leftover characters from a previous longer line.
                int pad = w - static_cast<int>(s.size()) + 1;
                if (pad > 0) s += std::string(pad, ' ');

                std::cout << s << std::flush;
                std::this_thread::sleep_for(std::chrono::milliseconds(80));
            }
            // Erase the spinner line before the result is printed.
            std::cout << "\r" << std::string(w, ' ') << "\r" << std::flush;
        });
    }

#ifdef __linux__
    PerfCounters perf;
    perf.reset_and_enable();
#endif

    solver.solve();
    auto t1 = std::chrono::steady_clock::now();

#ifdef __linux__
    perf.disable_and_read(stats);
#endif

    if (show_spinner) {
        done = true;
        spinner_thread.join();
    }

    stats.total_time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    stats.memory_peak_kb = utils::get_peak_memory_kb();

    return {description, stats, "", 0};
}

// ─── Config / Sweep Structs ───────────────────────────────────────────────────

struct BenchmarkConfig {
    std::ostream& out;
    bool file_output;
    bool collect_metrics;
    double focal_w;
    bool use_h_max;
};

struct SweepParams {
    std::vector<uint32_t> alphas;
    std::vector<uint32_t> betas;
    std::vector<int> ds;
};

struct AlgoGroups {
    std::vector<std::string> once_off;   // no parameter sweep
    std::vector<std::string> d_sweep;    // sweep over D values only
    std::vector<std::string> full_sweep; // sweep over alpha, beta, and D
};

// ─── Utilities ────────────────────────────────────────────────────────────────

std::vector<int> parse_range(const std::string& s) {
    std::vector<int> out;
    if (s.empty()) return out;
    size_t dash = s.find('-');
    if (dash != std::string::npos) {
        int lo = std::stoi(s.substr(0, dash));
        int hi = std::stoi(s.substr(dash + 1));
        for (int i = lo; i <= hi; ++i) out.push_back(i);
    } else {
        out.push_back(std::stoi(s));
    }
    return out;
}

template<typename T>
std::vector<T> parse_list(const std::string& s) {
    std::vector<T> result;
    if (s.empty()) return result;
    std::stringstream ss(s);
    std::string item;
    while (std::getline(ss, item, ',')) {
        if (item.empty()) continue;
        if constexpr (std::is_same_v<T, std::string>)  result.push_back(item);
        else if constexpr (std::is_integral_v<T>)       result.push_back(static_cast<T>(std::stoll(item)));
        else                                             result.push_back(static_cast<T>(std::stod(item)));
    }
    if (result.empty()) {
        if constexpr (std::is_same_v<T, std::string>)  result.push_back(s);
        else if constexpr (std::is_integral_v<T>)       result.push_back(static_cast<T>(std::stoll(s)));
        else                                             result.push_back(static_cast<T>(std::stod(s)));
    }
    return result;
}

// ─── Algorithm Dispatch ───────────────────────────────────────────────────────

// Runs a BucketHeap benchmark across the supported compile-time D values.
// `run` is a lambda that takes a heap by reference and executes the benchmark.
template<typename Calculator, typename RunFn>
void dispatch_d(int d, Calculator calc, uint32_t alpha, uint32_t beta, RunFn&& run) {
    switch (d) {
        case 2: { BucketHeap<Calculator, std::less<double>, 2> h(calc, alpha, beta); run(h); break; }
        case 4: { BucketHeap<Calculator, std::less<double>, 4> h(calc, alpha, beta); run(h); break; }
        case 8: { BucketHeap<Calculator, std::less<double>, 8> h(calc, alpha, beta); run(h); break; }
        default: std::cerr << "Unsupported D value for BucketHeap: " << d << "\n";
    }
}

template<typename Environment>
void execute_benchmarks(
    Environment& env,
    const std::vector<std::string>& algorithms,
    uint32_t alpha, uint32_t beta, int d,
    const BenchmarkConfig& cfg,
    const std::string& env_name,
    int instance_id)
{
    bool spinner = !cfg.file_output;

    auto emit = [&](BenchmarkResult result) {
        result.environment_name = env_name;
        result.instance_id = instance_id;
        if (cfg.file_output) write_csv_row(cfg.out, result);
        else                  print_row(cfg.out, result);
    };

    for (const auto& algo : algorithms) {
        if (algo == "astar_binary") {
            IndexedDaryHeap<uint32_t, 2> heap;
            emit(run_benchmark<AStar>(env, heap, "A* with IndexedHeap (D=2)", spinner, cfg.collect_metrics));

        } else if (algo == "astar_bucket") {
            BucketQueue queue;
            emit(run_benchmark<AStar>(env, queue, "A* with BucketQueue", spinner, cfg.collect_metrics));

        } else if (algo == "anytime_astar_binary") {
            IndexedDaryHeap<uint32_t, 2> heap;
            emit(run_benchmark<AnytimeAStar>(env, heap, "Anytime A* with IndexedHeap (D=2)", spinner, cfg.collect_metrics, cfg.focal_w));

        } else if (algo == "anastar_binary") {
            IndexedDaryHeap<double, 2, std::less<double>> heap;
            emit(run_benchmark<ANAStar>(env, heap, "ANA* with IndexedHeap (D=2)", spinner, cfg.collect_metrics));

        } else if (algo == "dps_binary") {
            IndexedDaryHeap<double, 2, std::less<double>> heap;
            emit(run_benchmark<DynamicPotentialSearch>(env, heap, "DPS with IndexedHeap (D=2)", spinner, cfg.collect_metrics, cfg.focal_w));

        } else if (algo == "anastar_bucket") {
            ANAStarPriorityCalculator calc;
            std::string desc = "ANA* with BucketHeap (D=" + std::to_string(d) + ")";
            auto run = [&](auto& h) { emit(run_benchmark<ANAStar>(env, h, desc, spinner, cfg.collect_metrics)); };
            switch (d) {
                case 2: { BucketHeap<ANAStarPriorityCalculator, std::less<double>, 2> h(calc); run(h); break; }
                case 4: { BucketHeap<ANAStarPriorityCalculator, std::less<double>, 4> h(calc); run(h); break; }
                case 8: { BucketHeap<ANAStarPriorityCalculator, std::less<double>, 8> h(calc); run(h); break; }
                default: std::cerr << "Unsupported D value for BucketHeap: " << d << "\n";
            }

        } else if (algo == "dps_bucket") {
            DPSPriorityCalculator calc;
            std::string desc = "DPS with BucketHeap (D=" + std::to_string(d) + ")";
            auto run = [&](auto& h) { emit(run_benchmark<DynamicPotentialSearch>(env, h, desc, spinner, cfg.collect_metrics, cfg.focal_w)); };
            switch (d) {
                case 2: { BucketHeap<DPSPriorityCalculator, std::less<double>, 2> h(calc); run(h); break; }
                case 4: { BucketHeap<DPSPriorityCalculator, std::less<double>, 4> h(calc); run(h); break; }
                case 8: { BucketHeap<DPSPriorityCalculator, std::less<double>, 8> h(calc); run(h); break; }
                default: std::cerr << "Unsupported D value for BucketHeap: " << d << "\n";
            }

        } else if (algo == "anastar_realbucket") {
            ANAStarPriorityCalculator calc;
            std::string desc = "ANA* with BucketHeap (a=" + std::to_string(alpha) + ", b=" + std::to_string(beta)
                             + ", D=" + std::to_string(d) + (cfg.use_h_max ? ", use_h_max" : "") + ")";
            auto run = [&](auto& h) { h.set_use_h_max(cfg.use_h_max); emit(run_benchmark<ANAStar>(env, h, desc, spinner, cfg.collect_metrics)); };
            dispatch_d(d, calc, alpha, beta, run);

        } else if (algo == "dps_realbucket") {
            DPSPriorityCalculator calc;
            std::string desc = "DPS with BucketHeap (a=" + std::to_string(alpha) + ", b=" + std::to_string(beta)
                             + ", D=" + std::to_string(d) + (cfg.use_h_max ? ", use_h_max" : "") + ")";
            auto run = [&](auto& h) { h.set_use_h_max(cfg.use_h_max); emit(run_benchmark<DynamicPotentialSearch>(env, h, desc, spinner, cfg.collect_metrics, cfg.focal_w)); };
            dispatch_d(d, calc, alpha, beta, run);

        } else if (algo == "astar_agg_two_level") {
            TwoLevelBucketQueue queue(alpha, beta);
            std::string desc = "A* with TwoLevelBucketQueue (a=" + std::to_string(alpha) + ", b=" + std::to_string(beta) + ")";
            emit(run_benchmark<AStar>(env, queue, desc, spinner, cfg.collect_metrics));
        }
    }
}

// ─── Sweep Runner ─────────────────────────────────────────────────────────────

template<typename Environment>
void run_sweep(Environment& env, const AlgoGroups& groups, const SweepParams& params,
               const BenchmarkConfig& cfg, const std::string& env_name, int instance_id) {
    if (!groups.once_off.empty())
        execute_benchmarks(env, groups.once_off, 0, 0, 0, cfg, env_name, instance_id);

    if (!groups.d_sweep.empty())
        for (int d : params.ds)
            execute_benchmarks(env, groups.d_sweep, 0, 0, d, cfg, env_name, instance_id);

    if (!groups.full_sweep.empty())
        for (int d : params.ds)
            for (uint32_t a : params.alphas)
                for (uint32_t b : params.betas)
                    execute_benchmarks(env, groups.full_sweep, a, b, d, cfg, env_name, instance_id);
}

// ─── Main ─────────────────────────────────────────────────────────────────────

// Sequences from Ikeda & Imai (1994), Fig. 2.
static const std::vector<std::string> kMSASequences = {
    "MSDEQHQNLAIIGHVDHGKSTLVGRLLYETGSVPEHVIEQHKEEAEEKGKGGFEFAYVMDNLAEERERGVTIDIAHQEFSTDTYDFTIVDCPGHRDFVKNMITGASQADNAVLVVAADGVQPQTQEHVFLARTLGIGELIVAVNKMDLVDYGESEYKQVVEEVKDLLTQVRFDSENAKFIPVSAFEGDNIAEESEHTGWYDGEILLEALNELPAPEPPTDAPLRLPIQDVYTISGIGTVPVGRVETGILNTGDNVSFQPDSVSGEVKTVEMHHEEVPKAEPGDNVGFNVRGVGKDDIRRGDVCGPADDPPSVAETFQAQIVVMQHPSVITEGYTPVFHAHTAQVACTVESIDKKIDPSSGEVAEENPDFIQNGDAAVVTVRPQKPLSIEPSSEIPELGSFAIRDMGQTIAAGKVLGVNER", // Hal
    "MAKTKPILNVAFIGHVDAGKSTTVGRLLLDGGAIDPQLIVRLRKEAEEKGKAGFEFAYVMDGLKEERERGVTIDVAHKKFPPAKYEVTIVDCPGHRDFIKNMITGASQADAAVLVVNVDDAKSGIQPQTREHVFLIRTLGVRQLAVAVNKMDTVNFSEADYNELKKMIGDQLLKMIGFNPEQINFVPVASLHGDNVFKKSERNPWYKGPTIAEVIDGFQPPEKPTNLPLRLPIQDVYTITGVGTVPVGRVETGIIKPGDKVVFEGAGEIKTVEMHHEQLPSAEPGDNIGFNVRGVGKKDIKRGDVLGHTTNPPTVATDFTAQIVVLQHPSVLTDGYTPVFHTHTAQIACTFAEIQKKLNPATGEVLEENPDFLKAGDAAIVKLIPTKPMVIESVKEIPQLGRFAIRDMGMTVAAGMAIQVTAKNK", // Met
    "MASQKPHLNLITIGHVDHGKSTLVGRLLYEHGEIPAHIIEEYRKEAEQKGKATFEFAWVMDRFKEERERGVTIDLAHRKFETDKYYFTLIDAPGHRDFVKNMITGTSQADAAILVISARDGEGVMEQTREHAFLARTLGVPQMVVAINKMDATSPPYSEKRYNEVKADAEKLLRSIGFKDISFVPISGYKGDNVTKPSPNMPWYKGPTLLQALDAFKVPEKPINKPLRIPVEDVYSITGIGTVPVGRVETGVLKPGDKVIFLPADKQGDVKSIEMHHEPLQQAEPGDNIGFNVRGIAKNDIKRGDVCGHLDTPPTVVKAFTAQIIVLNHPSVIAPGYKPVFHVHTAQVACRIDEIVKTLNPKDGTTLEKPDFIKNGDVAIVKVIPDKPLVIEKVSEIPQLGRFAVLDMGQTVAAGQCIDLEKR", // Tha
    "MAKEKPHINIVFIGHVDHGKSTTIGRLLFDTANIPENIIKKFEEMGEKGKSFKFAWVMDRLKEERERGITIDVAHTKFETPHRYITIIDAPGHRDFVKNMITGASQADAAVLVVAVTDGVMPQTKEHAFLARTLGINNILVAVNKMDMVNYDEKKFKAVAEQVKKLLMMLGYKNFPIIPISAWEGDNVVKKSDKMPWYNGPTLIEALDQMPEPPKPTDKPLRIPIQDVYSIKGVGTVPVGRVETGVLRVGDVVIFEPASTIFHKPIQGEVKSIEMHHEPMQEALPGDNIGFNVRGVGKNDIKRGDVAGHTNNPPTVVRPKDTFKAQIIVLNHPTAITVGYTPVLHAHTLQVAVRFEQLLAKLDPRTGNIVEENPQFIKTGDSAIVVLRPTKPMVIEPVKEIPQMGRFAIRDMGQTVAAGMVISIQKAE", // The
    "MSQKPHLNLIVIGHVDHGKSTLIGRLLMDRGFIDEKTVKEAEEAAKKLGKDSEKYAFLMDRLKEERERGVTINLSFMRFETRKYFFTVIDAPGHRDFVKNMITGASQADAAILVVSAKKGEYEAGMSAEGQTREHIILSKTMGINQVIVAINKMDLADTPYDEKRFKEIVDTVSKFMKSFGFDMNKVKFVPVVAPDGDNVTHKSTKMPWYNGPTLEELLDQLEIPPKPVDKPLRIPIQEVYSISGVGVVPVGRIESGVLKVGDKIVFMPVGKIGEVRSIETHHTKIDKAEPGDNIGFNVRGVEKKDVKRGDVAGSVQNPPTVADEFTAQVIVIWHPTAVGVGYTPVLHVHTASIACRVSEITSRIDPKTGKEAEKNPQFIKAGDSAIVKFKPIKELVAEKFREFPALGRFAMRDMGKTVGVGVIIDVKPRKVEVK", // Sul
    "MPKEKTHINIVVIGHVDSGKSTTTGHLIYKCGGIDQRTIEKFEKESAEMGKGSFKYAWVLDNLKAERERGITIDISLWKFETSKYYFTIIDAPGHRDFIKNMITGTSQADVAILIVAAGTGEFEAGISKNGQTREHILLSYTLGVKQMIVGVNKMDAIQYKQERYEEIKKEISAFLKKTGYNPDKIPFVPISGFQGDNMIEPSTNMPWYKGPTLIGALDSVTPPERPVDKPLRLPLQDVYKISGIGTVPVGRVETGILKPGTIVQFAPSGVSSECKSIEMHHTALAQAIPGDNVGFNVRNLTVKDIKRGNVASDAKNQPAVGCEDFTAQVIVMNHPGQIRKGYTPVLDCHTSHIACKFEELLSKIDRRTGKSMEGGEPEYIKNGDSALVKIVPTKPLCVEEFAKFPPLGRFAVRDMKQTVAVGVVKAVTP"  // Ent
};

int main(int argc, char** argv) {
    cxxopts::Options options("Benchmark", "Benchmarking tool for heuristic search algorithms.");

    options.add_options()
        ("p,pancakes",          "Number of pancakes",                         cxxopts::value<int>()->default_value("48"))
        ("grid-width",          "Grid width",                                  cxxopts::value<int>()->default_value("1000"))
        ("grid-height",         "Grid height",                                 cxxopts::value<int>()->default_value("1000"))
        ("grid-max-edge-cost",  "Grid max edge cost",                          cxxopts::value<uint32_t>()->default_value("1"))
        ("k,korf",              "Korf100 puzzle range (e.g. 0-9 or 5)",        cxxopts::value<std::string>()->default_value("0"))
        ("o,output",            "Output CSV file",                             cxxopts::value<std::string>())
        ("metrics",             "Collect detailed bucket metrics",             cxxopts::value<bool>()->default_value("false"))
        ("num-grid",            "Number of grid instances",                    cxxopts::value<int>()->default_value("1"))
        ("num-pancake",         "Number of pancake instances",                 cxxopts::value<int>()->default_value("1"))
        ("grid-alphas",         "CSV of Grid alpha values",                    cxxopts::value<std::string>()->default_value("1"))
        ("grid-betas",          "CSV of Grid beta values",                     cxxopts::value<std::string>()->default_value("10"))
        ("grid-ds",             "CSV of Grid D values",                        cxxopts::value<std::string>()->default_value("2"))
        ("pancake-alphas",      "CSV of Pancake alpha values",                 cxxopts::value<std::string>()->default_value("1"))
        ("pancake-betas",       "CSV of Pancake beta values",                  cxxopts::value<std::string>()->default_value("2"))
        ("pancake-ds",          "CSV of Pancake D values",                     cxxopts::value<std::string>()->default_value("2"))
        ("korf-alphas",         "CSV of Korf alpha values",                    cxxopts::value<std::string>()->default_value("1"))
        ("korf-betas",          "CSV of Korf beta values",                     cxxopts::value<std::string>()->default_value("10"))
        ("korf-ds",             "CSV of Korf D values",                        cxxopts::value<std::string>()->default_value("2"))
        ("msa-alphas",          "CSV of MSA alpha values",                     cxxopts::value<std::string>()->default_value("1"))
        ("msa-betas",           "CSV of MSA beta values",                      cxxopts::value<std::string>()->default_value("1"))
        ("msa-ds",              "CSV of MSA D values",                         cxxopts::value<std::string>()->default_value("2"))
        ("num-msa",             "Number of MSA instances (max 6)",             cxxopts::value<int>()->default_value("6"))
        ("capacity",            "Initial node pool capacity (grows as needed)", cxxopts::value<uint32_t>()->default_value("1000000"))
        ("non-heavy-heuristic", "Use unit-cost heuristic for heavy envs",      cxxopts::value<bool>()->default_value("false"))
        ("focal-w",             "Suboptimality bound for focal/anytime search", cxxopts::value<double>()->default_value("1.5"))
        ("use-h-max",           "Use h-max as bucket representative",          cxxopts::value<bool>()->default_value("false"))
        ("e,environments",      "Comma-separated environments to run",
            cxxopts::value<std::string>()->default_value("grid,pancake,korf100"))
        ("l,algorithms",        "Comma-separated algorithms to run",
            cxxopts::value<std::string>()->default_value(
                "astar_binary,astar_bucket,anytime_astar_binary,anastar_binary,"
                "anastar_bucket,anastar_realbucket,astar_agg_two_level"))
        ("h,help", "Print usage");

    auto args = options.parse(argc, argv);
    if (args.count("help")) {
        std::cout << options.help() << "\n";
        return 0;
    }

    // ── Output setup ────────────────────────────────────────────────────────
    std::ofstream outfile;
    bool file_output = args.count("output") > 0;
    std::ostream* out_ptr;

    if (file_output) {
        outfile.open(args["output"].as<std::string>());
        if (!outfile.is_open()) {
            std::cerr << "Error: Could not open output file: " << args["output"].as<std::string>() << "\n";
            return 1;
        }
        out_ptr = &outfile;
        write_csv_header(*out_ptr);
    } else {
        out_ptr = &std::cout;
        *out_ptr << "\033[2J\033[H";
    }

    BenchmarkConfig cfg{
        *out_ptr,
        file_output,
        args["metrics"].as<bool>(),
        args["focal-w"].as<double>(),
        args["use-h-max"].as<bool>()
    };

    // ── Algorithm grouping ───────────────────────────────────────────────────
    std::vector<std::string> all_algos;
    std::string algo_str = args["algorithms"].as<std::string>();
    if (algo_str == "all") {
        all_algos = {"astar_binary", "astar_bucket", "anytime_astar_binary", "anastar_binary",
                     "anastar_bucket", "anastar_realbucket", "astar_agg_two_level",
                     "dps_binary", "dps_bucket", "dps_realbucket"};
    } else {
        all_algos = parse_list<std::string>(algo_str);
    }

    AlgoGroups groups;
    for (const auto& algo : all_algos) {
        if (algo == "anastar_realbucket" || algo == "astar_agg_two_level" || algo == "dps_realbucket")
            groups.full_sweep.push_back(algo);
        else if (algo == "anastar_bucket" || algo == "dps_bucket")
            groups.d_sweep.push_back(algo);
        else
            groups.once_off.push_back(algo);
    }

    // ── Per-environment sweep parameters ────────────────────────────────────
    auto make_params = [&](const std::string& prefix) {
        return SweepParams{
            parse_list<uint32_t>(args[prefix + "-alphas"].as<std::string>()),
            parse_list<uint32_t>(args[prefix + "-betas"].as<std::string>()),
            parse_list<int>     (args[prefix + "-ds"].as<std::string>())
        };
    };
    SweepParams grid_params    = make_params("grid");
    SweepParams pancake_params = make_params("pancake");
    SweepParams korf_params    = make_params("korf");
    SweepParams msa_params     = make_params("msa");

    // ── Terminal output helpers ──────────────────────────────────────────────
    auto env_header = [&](const std::string& title, const std::string& sub = "") {
        if (!file_output) print_env_header(cfg.out, title, sub);
    };
    auto instance_header = [&](const std::string& label) {
        if (!file_output) {
            cfg.out << BOLD "  " << label << RESET "\n";
            print_header(cfg.out);
        }
    };
    auto instance_footer = [&]() {
        if (!file_output) cfg.out << "\n";
    };

    // ── Environment loop ─────────────────────────────────────────────────────
    int      num_grid        = args["num-grid"].as<int>();
    int      num_pancake     = args["num-pancake"].as<int>();
    int      num_msa         = args["num-msa"].as<int>();
    bool     heavy_heuristic = !args["non-heavy-heuristic"].as<bool>();
    uint32_t capacity        = args["capacity"].as<uint32_t>();

    for (const auto& env_name : parse_list<std::string>(args["environments"].as<std::string>())) {

        if (env_name == "grid") {
            env_header("Grid", std::to_string(args["grid-width"].as<int>()) + "×" + std::to_string(args["grid-height"].as<int>()));
            for (int i = 0; i < num_grid; ++i) {
                instance_header("Instance #" + std::to_string(i));
                GridEnvironment env(args["grid-width"].as<int>(), args["grid-height"].as<int>(), 1 + i);
                run_sweep(env, groups, grid_params, cfg, env_name, i);
                instance_footer();
            }

        } else if (env_name == "random_grid") {
            env_header("Random Grid", std::to_string(args["grid-width"].as<int>()) + "×" + std::to_string(args["grid-height"].as<int>()));
            for (int i = 0; i < num_grid; ++i) {
                instance_header("Instance #" + std::to_string(i));
                RandomGridEnvironment env(args["grid-width"].as<int>(), args["grid-height"].as<int>(),
                                          args["grid-max-edge-cost"].as<uint32_t>(), 42 + i);
                run_sweep(env, groups, grid_params, cfg, env_name, i);
                instance_footer();
            }

        } else if (env_name == "pancake") {
            env_header("Pancake Puzzle", std::to_string(PancakeEnvironment::N) + " pancakes");
            for (int i = 0; i < num_pancake; ++i) {
                instance_header("Instance #" + std::to_string(i));
                PancakeEnvironment env(42 + i, capacity);
                env.generate_start_node();
                run_sweep(env, groups, pancake_params, cfg, env_name, i);
                instance_footer();
            }

        } else if (env_name == "heavy_pancake") {
            env_header("Heavy Pancake Puzzle", std::to_string(HeavyPancakeEnvironment::N) + " pancakes");
            for (int i = 0; i < num_pancake; ++i) {
                instance_header("Instance #" + std::to_string(i));
                HeavyPancakeEnvironment env(42 + i, capacity);
                env.set_heavy_heuristic(heavy_heuristic);
                env.generate_start_node();
                run_sweep(env, groups, pancake_params, cfg, env_name, i);
                instance_footer();
            }

        } else if (env_name == "korf100") {
            env_header("Sliding Tile Puzzle", "Korf100");
            for (int idx : parse_range(args["korf"].as<std::string>())) {
                instance_header("Puzzle #" + std::to_string(idx));
                try {
                    SlidingTileEnvironment env(idx, "korf100.txt", capacity);
                    run_sweep(env, groups, korf_params, cfg, env_name, idx);
                } catch (const std::exception& e) {
                    std::cerr << "korf100 puzzle #" << idx << ": " << e.what() << "\n";
                }
                instance_footer();
            }

        } else if (env_name == "heavy_korf100") {
            env_header("Heavy Sliding Tile Puzzle", "Korf100");
            for (int idx : parse_range(args["korf"].as<std::string>())) {
                instance_header("Puzzle #" + std::to_string(idx));
                try {
                    HeavySlidingTileEnvironment env(idx, "korf100.txt", capacity);
                    env.set_heavy_heuristic(heavy_heuristic);
                    run_sweep(env, groups, korf_params, cfg, env_name, idx);
                } catch (const std::exception& e) {
                    std::cerr << "heavy_korf100 puzzle #" << idx << ": " << e.what() << "\n";
                }
                instance_footer();
            }

        } else if (env_name == "msa" || env_name == "msa5") {
            env_header("Multiple Sequence Alignment", "5-of-6 EF-TU/EF-1α proteins");
            for (int i = 0; i < std::min(num_msa, 6); ++i) {
                instance_header("Instance #" + std::to_string(i) + "  (excluding seq " + std::to_string(i) + ")");
                std::vector<std::string> seqs;
                for (int j = 0; j < 6; ++j)
                    if (j != i) seqs.push_back(kMSASequences[j]);
                MSA5Environment env(seqs, capacity);
                run_sweep(env, groups, msa_params, cfg, env_name, i);
                instance_footer();
            }

        } else if (env_name == "msa6") {
            env_header("Multiple Sequence Alignment", "All 6 EF-TU/EF-1α proteins");
            MSA6Environment env(kMSASequences, capacity);
            run_sweep(env, groups, msa_params, cfg, env_name, 0);
            instance_footer();
        }
    }

    if (file_output) {
        outfile.close();
        std::cout << "Benchmark results written to " << args["output"].as<std::string>() << "\n";
    }

    return 0;
}
