#include <iostream>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <fstream>

// Algorithms
#include "algorithms/a_star.h"
#include "algorithms/anytime_a_star.h"
#include "algorithms/ana_star.h"
#include "environments/environments.h"
#include "queues/binary_heap.h"
#include "queues/bucket_queue.h"
#include "queues/indexed_binary_heap.h"
#include "queues/bucket_heap.h"
#include "queues/two_level_bucket_queue.h"
#include "utils/utils.h"

template <template<typename, typename> class Algorithm, typename Environment, typename Queue>
void run_benchmark(Environment& env, Queue& queue, utils::SearchStats& stats, const std::string& description) {
    std::cout << description << "\n\n";
    stats.reset();
    env.reset_search();
    queue.clear(); 

    using ProfiledQueueType = utils::ProfiledQueue<Queue>;
    ProfiledQueueType profiled_queue(queue, stats);

    using Solver = Algorithm<Environment, ProfiledQueueType>;
    Solver solver(env, profiled_queue, &stats);

    auto start_time = std::chrono::high_resolution_clock::now();
    solver.solve();
    auto end_time = std::chrono::high_resolution_clock::now();
    stats.total_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
    stats.memory_peak_kb = utils::get_peak_memory_kb();

    utils::print_stats(stats);
    std::cout << "-----------------------\n\n";
}

int main() {
    utils::SearchStats stats;

    std::cout << "\n\033[1m" << "Grid Environment" << "\033[0m\n" << "========================\n\n";
    GridEnvironment grid_env(1000, 1000, 42);

    {
        BinaryHeap<uint32_t> heap;
        run_benchmark<AStar>(grid_env, heap, stats, "A* with BinaryHeap");
    }

    {
        BucketQueue bq;
        run_benchmark<AStar>(grid_env, bq, stats, "A* with BucketQueue");
    }
    
    {
        BinaryHeap<uint32_t> heap;
        run_benchmark<AnytimeAStar>(grid_env, heap, stats, "Anytime A* with BinaryHeap");
    }

    {
        BinaryHeap<double, std::less<double>> heap;
        run_benchmark<ANAStar>(grid_env, heap, stats, "ANA* with BinaryHeap");
    }

    {
        struct ANAStarPriorityCalculator {
            double G_upper = std::numeric_limits<double>::max();

            void set_g_upper(double g) {
                G_upper = g;
            }

            double operator()(uint32_t f, uint32_t h) const {
                if (h == 0) {
                    return std::numeric_limits<double>::max();
                }
                double g = static_cast<double>(f) - h;
                if (g >= G_upper) {
                    return std::numeric_limits<double>::lowest();
                }
                return (G_upper - g) / static_cast<double>(h);
            }
        };
        ANAStarPriorityCalculator calculator;
        BucketHeap<ANAStarPriorityCalculator, std::less<double>> bucket_heap(calculator);
        run_benchmark<ANAStar>(grid_env, bucket_heap, stats, "ANA* with BucketHeap");
    }

    std::cout << "\n\033[1m" << "Sliding Tile Puzzle (Easy)" << "\033[0m\n" << "========================\n\n";

    try {
        SlidingTileEnvironment tile_env(0, "easy.txt", 20000000);

        {
            BinaryHeap<uint32_t> heap;
            run_benchmark<AStar>(tile_env, heap, stats, "A* with BinaryHeap");
        }
        
        {
            BinaryHeap<uint32_t> heap;
            run_benchmark<AnytimeAStar>(tile_env, heap, stats, "Anytime A* with BinaryHeap");
        }
        
        {
            BinaryHeap<double, std::less<double>> heap;
            run_benchmark<ANAStar>(tile_env, heap, stats, "ANA* with BinaryHeap");
        }

        {
            struct ANAStarPriorityCalculator {
                double G_upper = std::numeric_limits<double>::max();

                void set_g_upper(double g) {
                    G_upper = g;
                }

                double operator()(uint32_t f, uint32_t h) const {
                    if (h == 0) {
                        return std::numeric_limits<double>::max();
                    }
                    double g = static_cast<double>(f) - h;
                    if (g >= G_upper) {
                        return std::numeric_limits<double>::lowest();
                    }
                    return (G_upper - g) / static_cast<double>(h);
                }
            };
            ANAStarPriorityCalculator calculator;
            BucketHeap<ANAStarPriorityCalculator, std::less<double>> bucket_heap(calculator);
            run_benchmark<ANAStar>(tile_env, bucket_heap, stats, "ANA* with BucketHeap");
        }

    } catch (const std::exception& e) {
        std::cerr << "Error setting up SlidingTileEnvironment: " << e.what() << std::endl;
    }

    std::cout << "\n\033[1m" << "Sliding Tile Puzzle (Korf100)" << "\033[0m\n" << "========================\n\n";
    
    try {
        SlidingTileEnvironment tile_env(0, "korf100.txt", 20000000);

        {
            BinaryHeap<uint32_t> heap;
            run_benchmark<AStar>(tile_env, heap, stats, "A* with BinaryHeap");
        }
        
        {
            BinaryHeap<uint32_t> heap;
            run_benchmark<AnytimeAStar>(tile_env, heap, stats, "Anytime A* with BinaryHeap");
        }
        
        {
            BinaryHeap<double, std::less<double>> heap;
            run_benchmark<ANAStar>(tile_env, heap, stats, "ANA* with BinaryHeap");
        }

        {
            struct ANAStarPriorityCalculator {
                double G_upper = std::numeric_limits<double>::max();

                void set_g_upper(double g) {
                    G_upper = g;
                }

                double operator()(uint32_t f, uint32_t h) const {
                    if (h == 0) {
                        return std::numeric_limits<double>::max();
                    }
                    double g = static_cast<double>(f) - h;
                    if (g >= G_upper) {
                        return std::numeric_limits<double>::lowest();
                    }
                    return (G_upper - g) / static_cast<double>(h);
                }
            };
            ANAStarPriorityCalculator calculator;
            BucketHeap<ANAStarPriorityCalculator, std::less<double>> bucket_heap(calculator);
            run_benchmark<ANAStar>(tile_env, bucket_heap, stats, "ANA* with BucketHeap");
        }

    } catch (const std::exception& e) {
        std::cerr << "Error setting up SlidingTileEnvironment: " << e.what() << std::endl;
    }

    std::cout << "\033[1m" << "Pancake Puzzle" << "\033[0m\n" << "========================\n\n";
    
    PancakeEnvironment pancake_env(42, 50000000);
    pancake_env.generate_start_node();

    {
        BinaryHeap<uint32_t> heap;
        run_benchmark<AStar>(pancake_env, heap, stats, "A* with BinaryHeap");
    }
    
    {
        BinaryHeap<uint32_t> heap;
        run_benchmark<AnytimeAStar>(pancake_env, heap, stats, "Anytime A* with BinaryHeap");
    }
    
    {
        BinaryHeap<double, std::less<double>> heap;
        run_benchmark<ANAStar>(pancake_env, heap, stats, "ANA* with BinaryHeap");
    }

    {
        struct ANAStarPriorityCalculator {
            double G_upper = std::numeric_limits<double>::max();

            void set_g_upper(double g) {
                G_upper = g;
            }

            double operator()(uint32_t f, uint32_t h) const {
                if (h == 0) {
                    return std::numeric_limits<double>::max();
                }
                double g = static_cast<double>(f) - h;
                if (g >= G_upper) {
                    return std::numeric_limits<double>::lowest();
                }
                return (G_upper - g) / static_cast<double>(h);
            }
        };
        ANAStarPriorityCalculator calculator;
        BucketHeap<ANAStarPriorityCalculator, std::less<double>> bucket_heap(calculator);
        run_benchmark<ANAStar>(pancake_env, bucket_heap, stats, "ANA* with BucketHeap");
    }

    return 0;
}
