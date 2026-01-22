#include "queues/binary_heap.h"
#include "algorithms/a_star.h"
#include "environments/environments.h"
#include "utils/utils.h"

int main() {

  GridEnvironment env(10, 10);
  BinaryHeap heap(env.get_pool());
  AStar<GridEnvironment, BinaryHeap> solver(env, heap);

  utils::SearchStatistics stats;

  // Measure execution time
  stats.execution_time = utils::measure_time([&](){
    solver.solve();
  });

  // Get peak memory usage (if supported by the platform)
  stats.peak_memory_bytes = utils::get_memory_usage_bytes();

  // For a basic check, we'll assume a solution is found and set a dummy cost/length.
  // In a real scenario, you would retrieve these from the solver after solve() completes.
  uint32_t goal_state_id = 10 * 10 - 1; // Assuming 10x10 grid, goal is 99
  uint32_t goal_handle = env.get_node_handle(goal_state_id);

  if (goal_handle != NODE_NULL) {
      stats.solution_cost = env.get_pool()[goal_handle].g;
      // You'd need to trace back parent pointers to get actual solution length
      // For now, setting a dummy length if a solution was found.
      stats.solution_length = 0; // Placeholder
      uint32_t current_node = goal_handle;
      while (env.get_pool()[current_node].parent != NODE_NULL) {
          stats.solution_length++;
          current_node = env.get_pool()[current_node].parent;
      }
      if (stats.solution_length > 0) { // Add 1 for the goal node itself if a path exists
          stats.solution_length++;
      }
  }

  // Nodes expanded would need to be tracked inside the AStar algorithm
  // For this example, we'll leave it at 0.

  utils::print_stats(stats);

  return 0;
}
