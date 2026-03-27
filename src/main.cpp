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

#include "algorithms/a_star.h"
#include "algorithms/anytime_a_star.h"
#include "algorithms/ana_star.h"
#include "algorithms/dps.h"
#include "environments/environments.h"
#include "queues/bucket_queue.h"
#include "queues/bucket_heap.h"
#include "queues/two_level_bucket_queue.h"
#include "utils/utils.h"

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
        << "total_hmin_scans,total_secondary_bucket_allocs\n";
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
        << s.total_hmin_scans << "," << s.total_secondary_bucket_allocs << "\n";
}

// ─── Table Output ─────────────────────────────────────────────────────────────

int get_terminal_width() {
    winsize size;
    ioctl(STDOUT_FILENO, TIOCGWINSZ, &size);
    return size.ws_col;
}

void print_header(std::ostream& out) {
    out << "\033[1m" << std::left
        << std::setw(45) << "Algorithm"
        << std::setw(15) << "Sol Cost"
        << std::setw(15) << "Expanded"
        << std::setw(15) << "Generated"
        << std::setw(15) << "Time (ms)"
        << std::setw(15) << "Mem (KB)"
        << "\033[0m\n"
        << std::string(get_terminal_width(), '-') << "\n";
}

void print_row(std::ostream& out, const BenchmarkResult& r) {
    const auto& s = r.stats;
    out << std::left
        << std::setw(45) << r.description
        << std::setw(15) << s.solution_cost
        << std::setw(15) << s.nodes_expanded
        << std::setw(15) << s.nodes_generated
        << std::setw(15) << std::fixed << std::setprecision(3) << s.total_time_ms
        << std::setw(15) << s.memory_peak_kb
        << "\n";

    double avg_enq = s.count_enqueue     > 0 ? s.time_enqueue     / s.count_enqueue     : 0;
    double avg_deq = s.count_dequeue     > 0 ? s.time_dequeue     / s.count_dequeue     : 0;
    double avg_reb = s.count_rebuild     > 0 ? s.time_rebuild     / s.count_rebuild     : 0;
    double avg_dk  = s.count_decrease_key > 0 ? s.time_decrease_key / s.count_decrease_key : 0;
    double wasted_ms = (avg_deq * s.count_stale_pops) / 1e6;

    out << "  \033[2mQueue Ops -> "
        << "Enq: "   << s.count_enqueue     << " (" << std::fixed << std::setprecision(1) << avg_enq << " ns), "
        << "Deq: "   << s.count_dequeue     << " (" << avg_deq << " ns), "
        << "D-Key: " << s.count_decrease_key << " (" << avg_dk  << " ns), "
        << "Reb: "   << s.count_rebuild     << " (" << avg_reb << " ns), "
        << "Total Overhead: " << std::fixed << std::setprecision(3)
        << (s.time_enqueue + s.time_dequeue + s.time_rebuild + s.time_decrease_key) / 1e6 << " ms"
        << "\033[0m\n"
        << "  \033[2mWaste     -> "
        << "Stale Pops: "    << s.count_stale_pops   << ", "
        << "Update Pushes: " << s.count_update_pushes << ", "
        << "Wasted Time: "   << std::fixed << std::setprecision(3) << wasted_ms << " ms"
        << "\033[0m\n"
        << "  \033[2mInternal  -> "
        << "H-Min Scans: "   << s.total_hmin_scans << ", "
        << "Bucket Allocs: " << s.total_secondary_bucket_allocs
        << "\033[0m\n";
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
    if (show_spinner) {
        spinner_thread = std::thread([&]() {
            const char frames[] = "|/-\\";
            int i = 0;
            while (!done) {
                std::cout << "\rRunning... " << frames[i++ % 4] << std::flush;
                std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
            std::cout << "\r" << std::string(20, ' ') << "\r";
        });
    }

    auto t0 = std::chrono::steady_clock::now();
    solver.solve();
    auto t1 = std::chrono::steady_clock::now();

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
    int tw = get_terminal_width();
    auto print_env_header = [&](const std::string& title) {
        if (!file_output)
            cfg.out << "\n\033[1m" << title << "\033[0m\n" << std::string(tw, '=') << "\n";
    };
    auto print_instance_header = [&](const std::string& label) {
        if (!file_output) {
            cfg.out << "\033[1;34m" << label << "\033[0m\n";
            print_header(cfg.out);
        }
    };
    auto print_instance_footer = [&]() {
        if (!file_output) cfg.out << "\n";
    };

    // ── Environment loop ─────────────────────────────────────────────────────
    int num_grid    = args["num-grid"].as<int>();
    int num_pancake = args["num-pancake"].as<int>();
    int num_msa     = args["num-msa"].as<int>();
    bool heavy_heuristic = !args["non-heavy-heuristic"].as<bool>();

    for (const auto& env_name : parse_list<std::string>(args["environments"].as<std::string>())) {

        if (env_name == "grid") {
            print_env_header("Grid Environment (" + std::to_string(args["grid-width"].as<int>())
                           + "x" + std::to_string(args["grid-height"].as<int>()) + ")");
            for (int i = 0; i < num_grid; ++i) {
                print_instance_header("Running instance #" + std::to_string(i));
                GridEnvironment env(args["grid-width"].as<int>(), args["grid-height"].as<int>(), 1 + i);
                run_sweep(env, groups, grid_params, cfg, env_name, i);
                print_instance_footer();
            }

        } else if (env_name == "random_grid") {
            print_env_header("Random Grid Environment (" + std::to_string(args["grid-width"].as<int>())
                           + "x" + std::to_string(args["grid-height"].as<int>()) + ")");
            for (int i = 0; i < num_grid; ++i) {
                print_instance_header("Running instance #" + std::to_string(i));
                RandomGridEnvironment env(args["grid-width"].as<int>(), args["grid-height"].as<int>(),
                                          args["grid-max-edge-cost"].as<uint32_t>(), 42 + i);
                run_sweep(env, groups, grid_params, cfg, env_name, i);
                print_instance_footer();
            }

        } else if (env_name == "pancake") {
            print_env_header("Pancake Puzzle (" + std::to_string(PancakeEnvironment::N) + " pancakes)");
            for (int i = 0; i < num_pancake; ++i) {
                print_instance_header("Running instance #" + std::to_string(i));
                PancakeEnvironment env(42 + i, 50000000);
                env.generate_start_node();
                run_sweep(env, groups, pancake_params, cfg, env_name, i);
                print_instance_footer();
            }

        } else if (env_name == "heavy_pancake") {
            print_env_header("Heavy Pancake Puzzle (" + std::to_string(HeavyPancakeEnvironment::N) + " pancakes)");
            for (int i = 0; i < num_pancake; ++i) {
                print_instance_header("Running instance #" + std::to_string(i));
                HeavyPancakeEnvironment env(42 + i, 50000000);
                env.set_heavy_heuristic(heavy_heuristic);
                env.generate_start_node();
                run_sweep(env, groups, pancake_params, cfg, env_name, i);
                print_instance_footer();
            }

        } else if (env_name == "korf100") {
            print_env_header("Sliding Tile Puzzle (Korf100)");
            for (int idx : parse_range(args["korf"].as<std::string>())) {
                print_instance_header("Running puzzle #" + std::to_string(idx));
                try {
                    SlidingTileEnvironment env(idx, "korf100.txt", 20000000);
                    run_sweep(env, groups, korf_params, cfg, env_name, idx);
                } catch (const std::exception& e) {
                    std::cerr << "Error setting up SlidingTileEnvironment for puzzle #" << idx << ": " << e.what() << "\n";
                }
                print_instance_footer();
            }

        } else if (env_name == "heavy_korf100") {
            print_env_header("Heavy Sliding Tile Puzzle (Korf100)");
            for (int idx : parse_range(args["korf"].as<std::string>())) {
                print_instance_header("Running puzzle #" + std::to_string(idx));
                try {
                    HeavySlidingTileEnvironment env(idx, "korf100.txt", 20000000);
                    env.set_heavy_heuristic(heavy_heuristic);
                    run_sweep(env, groups, korf_params, cfg, env_name, idx);
                } catch (const std::exception& e) {
                    std::cerr << "Error setting up HeavySlidingTileEnvironment for puzzle #" << idx << ": " << e.what() << "\n";
                }
                print_instance_footer();
            }

        } else if (env_name == "msa" || env_name == "msa5") {
            print_env_header("Multiple Sequence Alignment (5 of 6 EF-TU/EF-1a Proteins)");
            for (int i = 0; i < std::min(num_msa, 6); ++i) {
                print_instance_header("Running instance #" + std::to_string(i)
                                    + " (excluding sequence " + std::to_string(i) + ")");
                std::vector<std::string> seqs;
                for (int j = 0; j < 6; ++j)
                    if (j != i) seqs.push_back(kMSASequences[j]);
                MSA5Environment env(seqs, 50000000);
                run_sweep(env, groups, msa_params, cfg, env_name, i);
                print_instance_footer();
            }

        } else if (env_name == "msa6") {
            print_env_header("Multiple Sequence Alignment (All 6 EF-TU/EF-1a Proteins)");
            MSA6Environment env(kMSASequences, 50000000);
            run_sweep(env, groups, msa_params, cfg, env_name, 0);
            print_instance_footer();
        }
    }

    if (file_output) {
        outfile.close();
        std::cout << "Benchmark results written to " << args["output"].as<std::string>() << "\n";
    }

    return 0;
}
