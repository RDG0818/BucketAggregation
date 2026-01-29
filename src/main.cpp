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
    std::cout << description << "\n";
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

    std::cout << "Grid Environment\n";

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

    // std::cout << "========================================\n";
    // std::cout << "       Sliding Tile Puzzle (Easy)       \n";
    // std::cout << "========================================\n\n";

    // {
    //     std::ofstream easy_file("easy.txt");
    //     easy_file << "1 2 3 4 5 6 7 8 9 10 11 12 13 14 0 15\n"; 
    // }

    // try {
    //     SlidingTileEnvironment tile_env(0, "easy.txt");

    //     {
    //         BinaryHeap<uint32_t> heap;
    //         run_benchmark<AStar>(tile_env, heap, stats, "A* with BinaryHeap (Easy)");
    //     }
    // } catch (const std::exception& e) {
    //     std::cerr << "Error setting up easy SlidingTileEnvironment: " << e.what() << std::endl;
    // }

    // std::cout << "========================================\n";
    // std::cout << "       Sliding Tile Puzzle (Inst 0)     \n";
    // std::cout << " (This is a hard puzzle and may take a long time)\n";
    // std::cout << "========================================\n\n";
    
    // try {
    //     SlidingTileEnvironment tile_env(0, "korf100.txt");

    //     {
    //         BinaryHeap<uint32_t> heap;
    //         run_benchmark<AStar>(tile_env, heap, stats, "A* with BinaryHeap");
    //     }
        
    //     {
    //         BinaryHeap<uint32_t> heap;
    //         run_benchmark<AnytimeAStar>(tile_env, heap, stats, "Anytime A* with BinaryHeap");
    //     }
        
    //     {
    //         BinaryHeap<double, std::less<double>> heap;
    //         run_benchmark<ANAStar>(tile_env, heap, stats, "ANA* with BinaryHeap");
    //     }

    // } catch (const std::exception& e) {
    //     std::cerr << "Error setting up SlidingTileEnvironment: " << e.what() << std::endl;
    // }

    std::cout << "Pancake Puzzle (N=20)\n";
    
    PancakeEnvironment pancake_env(42);
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

    return 0;
}
