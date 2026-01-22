#include "queues/binary_heap.h"
#include "algorithms/a_star.h"
#include "algorithms/ana_star.h"
#include "environments/environments.h"
#include "utils/utils.h"
#include <iostream>

int main() {
    const uint32_t goal_state_id = 10 * 10 - 1;
    const uint32_t seed = 123; // Using a known solvable seed

    // --- Run A* with Profiling ---
    std::cout << "--- Running A* ---\\n";
    {
        utils::SearchStats stats;
        GridEnvironment env(10, 10, seed);

        // Define the underlying queue
        using AStarHeap = BinaryHeap<uint32_t, std::less<uint32_t>>;
        AStarHeap heap(env.get_pool());

        // Wrap it in the profiler
        utils::ProfiledQueue<AStarHeap> profiled_heap(heap, stats);

        // Pass the profiled queue and stats object to the solver
        AStar<GridEnvironment, decltype(profiled_heap)> solver(env, profiled_heap, &stats);

        const auto start_time = std::chrono::high_resolution_clock::now();
        solver.solve();
        const auto end_time = std::chrono::high_resolution_clock::now();
        stats.total_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();
        
        uint32_t goal_handle = env.get_node_handle(goal_state_id);
        if (goal_handle != NODE_NULL) {
            stats.solution_cost = env.get_pool()[goal_handle].g;
            uint32_t current_node = goal_handle;
            stats.solution_length = 1; // Start with 1 for the goal node itself
            while (env.get_pool()[current_node].parent != NODE_NULL) {
                stats.solution_length++;
                current_node = env.get_pool()[current_node].parent;
            }
        }
        utils::print_stats(stats);
    }

    // --- Run ANA* with Profiling ---
    std::cout << "\\n\\n--- Running ANA* ---\\n";
    {
        utils::SearchStats stats;
        GridEnvironment env(10, 10, seed);
        
        // Using float for priority as a cache-conscious choice
        using ANAStarHeap = BinaryHeap<float, std::greater<float>>;
        ANAStarHeap heap(env.get_pool());
        
        // Wrap it in the profiler
        utils::ProfiledQueue<ANAStarHeap> profiled_heap(heap, stats);

        // Pass the profiled queue and stats object to the solver
        ANAStar<GridEnvironment, decltype(profiled_heap)> solver(env, profiled_heap, &stats);

        const auto start_time = std::chrono::high_resolution_clock::now();
        solver.solve();
        const auto end_time = std::chrono::high_resolution_clock::now();
        stats.total_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        uint32_t goal_handle = env.get_node_handle(goal_state_id);
        if (goal_handle != NODE_NULL) {
            // The final cost is already set by the ANA* solver internally
            uint32_t current_node = goal_handle;
            stats.solution_length = 1; // Start with 1 for the goal node itself
            while (env.get_pool()[current_node].parent != NODE_NULL) {
                stats.solution_length++;
                current_node = env.get_pool()[current_node].parent;
            }
        }
        utils::print_stats(stats);
    }

    return 0;
}