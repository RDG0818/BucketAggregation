import timeit
import random
import sys
import argparse
from typing import Callable, Any
import itertools

# Import the algorithms to compare
from ana_with_binary_heap import anytime_nonparametric_astar as ana_binary_heap
from ana_with_bucket_heap import anytime_nonparametric_astar as ana_bucket_heap
from ana_with_bucket_heap_real import anytime_nonparametric_astar as ana_bucket_heap_real

INF = sys.maxsize
Node = tuple[int, int]
Grid = list[list[int]]

def generate_grid(rows: int, cols: int, obstacle_density: float, max_weight: int, start: Node, goal: Node) -> Grid:
    if max_weight < 1:
        max_weight = 1
    grid = [[random.randint(1, max_weight) for _ in range(cols)] for _ in range(rows)]
    
    guaranteed_path = set()
    r, c = start
    
    while c != goal[1]:
        guaranteed_path.add((r, c))
        c = c + 1 if c < goal[1] else c - 1
    
    while r != goal[0]:
        guaranteed_path.add((r, c))
        r = r + 1 if r < goal[0] else r - 1
        
    guaranteed_path.add(goal)

    for r in range(rows):
        for c in range(cols):
            if (r, c) in guaranteed_path:
                continue
            if random.random() < obstacle_density:
                grid[r][c] = 0
    
    grid[start[0]][start[1]] = 1
    grid[goal[0]][goal[1]] = 1
    
    return grid

def heuristic(a: Node, b: Node) -> int:
    """Manhattan distance heuristic."""
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def run_benchmark(algorithm: Callable, grid: Grid, start: Node, goal: Node, **kwargs: Any) -> dict:
    """Runs a single instance of an algorithm and collects metrics."""
    search_generator = algorithm(grid, start, goal, heuristic, **kwargs)
    
    final_result = None
    first_solution_result = None
    time_to_first_solution = None
    
    start_time = timeit.default_timer()
    
    try:
        while True:
            result = next(search_generator)
            if result.get("new_solution_found") and first_solution_result is None:
                time_to_first_solution = timeit.default_timer() - start_time
                first_solution_result = result
            if result.get("is_final"):
                final_result = result
                break
    except StopIteration:
        pass 
    except Exception as e:
        return {"error": str(e)}

    end_time = timeit.default_timer()
    total_time_taken = end_time - start_time

    if final_result is None:
         return {"error": "Algorithm did not yield a final result."}

    path_found = final_result.get("path") is not None
    
    if first_solution_result is None and path_found:
        first_solution_result = final_result
        time_to_first_solution = total_time_taken

    return {
        "total_time_sec": total_time_taken,
        "path_found": path_found,
        "final_path_cost": final_result.get("cost", INF),
        "final_expanded_nodes": final_result.get("expanded_nodes", 0),
        "time_to_first_solution_sec": time_to_first_solution,
        "first_solution_cost": first_solution_result.get("cost", INF) if first_solution_result else INF,
        "params": kwargs,
    }

def print_summary(alg_name: str, results: list[dict]):
    """Prints a formatted summary of benchmark results for an algorithm."""
    num_runs = len(results)
    if num_runs == 0:
        print(f"\n--- Summary for {alg_name} (0 Runs) ---")
        return

    print(f"\n--- Summary for {alg_name} ({num_runs} Successful Runs) ---")
    
    avg_time_optimal = (sum(r['total_time_sec'] for r in results) / num_runs) * 1000
    avg_cost_optimal = sum(r['final_path_cost'] for r in results) / num_runs
    avg_nodes_optimal = sum(r['final_expanded_nodes'] for r in results) / num_runs
    
    first_solution_results = [r for r in results if r['time_to_first_solution_sec'] is not None]
    if first_solution_results:
        avg_time_first = (sum(r['time_to_first_solution_sec'] for r in first_solution_results) / len(first_solution_results)) * 1000
        avg_cost_first = sum(r['first_solution_cost'] for r in first_solution_results) / len(first_solution_results)
        print(f"  Avg. Time to First Solution: {avg_time_first:.4f} ms")
        print(f"  Avg. First Solution Cost:    {avg_cost_first:.2f}")

    print(f"  Avg. Time to Optimal Solution: {avg_time_optimal:.4f} ms")
    print(f"  Avg. Optimal Solution Cost:    {avg_cost_optimal:.2f}")
    print(f"  Avg. Total Nodes Expanded:     {avg_nodes_optimal:.2f}")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Benchmark ANA* Heap Implementations")
    parser.add_argument('--rows', type=int, default=100, help='Grid rows')
    parser.add_argument('--cols', type=int, default=100, help='Grid columns')
    parser.add_argument('--density', type=float, default=0.3, help='Obstacle density')
    parser.add_argument('--max_weight', type=int, default=10, help='Max random cell weight')
    parser.add_argument('--runs', type=int, default=10, help='Number of comparative runs')
    parser.add_argument('--alphas', type=str, default="1.0", help='Comma-separated alpha values for Real Bucket Heap')
    parser.add_argument('--betas', type=str, default="1.0,2.0,5.0,10.0,15.0,20.0,25.0", help='Comma-separated beta values for Real Bucket Heap')
    
    args = parser.parse_args()

    alphas = [float(a) for a in args.alphas.split(',')]
    betas = [float(b) for b in args.betas.split(',')]

    START_NODE = (0, 0)
    GOAL_NODE = (args.rows - 1, args.cols - 1)
    
    # Define the baseline algorithms
    algorithms_to_test = [
        ("ANA* (Binary Heap)", ana_binary_heap, {}),
        ("ANA* (Bucket Heap)", ana_bucket_heap, {}),
    ]
    
    # Add the real-valued bucket heap with all combinations of alpha and beta
    for alpha, beta in itertools.product(alphas, betas):
        alg_name = f"ANA* (Real Bucket, a={alpha}, b={beta})"
        params = {"alpha": alpha, "beta": beta}
        algorithms_to_test.append((alg_name, ana_bucket_heap_real, params))

    print("--- Starting ANA* Heap Implementation Benchmark ---")
    print(f"Grid: {args.rows}x{args.cols}, Density: {args.density:.1f}, Max Weight: {args.max_weight}")
    print(f"Runs: {args.runs}\n")

    results_by_alg = {name: [] for name, _, _ in algorithms_to_test}
    
    for i in range(args.runs):
        print(f"--- Run {i+1}/{args.runs} ---")
        grid = generate_grid(args.rows, args.cols, args.density, args.max_weight, START_NODE, GOAL_NODE)
        
        for name, alg_func, params in algorithms_to_test:
            print(f"  Benchmarking {name}...")
            result = run_benchmark(alg_func, grid, START_NODE, GOAL_NODE, **params)
            
            if "error" in result or not result.get("path_found"):
                print(f"    - {name}: Failed or no path found.")
            else:
                results_by_alg[name].append(result)

    # --- Print final summary for all algorithms ---
    for name, _, _ in algorithms_to_test:
        results = results_by_alg[name]
        print_summary(name, results)
    
    print("\n--- Benchmark Complete ---")
