import pytest
import math
from ana_with_bucket_heap import anytime_nonparametric_astar, INF

@pytest.fixture
def sample_grid():
    """The standard 5x5 grid from the A* tests."""
    return [
        [1, 1, 1, 0, 1], # 0
        [1, 5, 5, 1, 1], # 1
        [1, 4, 3, 1, 1], # 2
        [0, 2, 1, 5, 0], # 3
        [1, 0, 1, 1, 1]  # 4
    ] # 0  1  2  3  4

@pytest.fixture
def trap_grid():
    """
    A grid designed to "trap" a greedy search.
    Start (0,3), Goal (4,3).
    Optimal path (cost 10) is to go left/right around the high-cost center.
    Suboptimal path (cost 20) is to go straight down the center.
    A greedy search might find the 20-cost path first.
    """
    return [
        [1, 1, 1, 1, 1, 1, 1], # 0
        [1, 9, 9, 9, 9, 9, 1], # 1
        [1, 1, 1, 1, 1, 1, 1], # 2
        [1, 9, 9, 9, 9, 9, 1], # 3
        [1, 1, 1, 1, 1, 1, 1]  # 4
    ] # 0  1  2  3  4  5  6

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def get_all_solutions(pathfinder_gen):
    """
    Consumes the ANA* generator and returns all solutions found.
    
    Returns:
        tuple: (list_of_solutions, final_solution)
        `list_of_solutions` contains all dicts with 'new_solution_found': True
        `final_solution` is the last dict, with 'is_final': True
    """
    solutions = []
    final_solution = None
    
    for step in pathfinder_gen:
        if step.get("new_solution_found", False):
            solutions.append(step)
        
        if step.get("is_final", False):
            final_solution = step
            break
            
    return solutions, final_solution

def get_final_result(pathfinder_gen):
    """Helper to get only the final, optimal result."""
    _, final = get_all_solutions(pathfinder_gen)
    return final

# --- Test Cases ---

def test_simple_path_optimal(sample_grid):
    """Tests ANA* on a simple path. It should find the optimal path."""
    start, goal = (0, 0), (0, 2)
    
    result = get_final_result(anytime_nonparametric_astar(sample_grid, start, goal, heuristic))
    
    assert result is not None
    assert result["path"] == [(0, 0), (0, 1), (0, 2)]
    assert math.isclose(result["cost"], 2.0)
    assert result["is_final"] == True
    assert result["expanded_nodes"] > 0

def test_weighted_path_optimal(sample_grid):
    """Tests ANA* on a path with weights. It should find the optimal path."""
    start, goal = (0, 0), (2, 2)
    
    result = get_final_result(anytime_nonparametric_astar(sample_grid, start, goal, heuristic))
    
    assert result is not None
    assert result["path"] == [(0, 0), (1, 0), (2, 0), (2, 1), (2,2)]
    assert math.isclose(result["cost"], 9.0)

def test_no_path_obstacle_goal(sample_grid):
    """Tests ANA* when the goal is an obstacle."""
    start, goal = (0, 0), (0, 3)
    
    result = get_final_result(anytime_nonparametric_astar(sample_grid, start, goal, heuristic))
    
    assert result is not None
    assert result["path"] is None
    assert result["cost"] == INF

def test_no_path_fully_blocked(sample_grid):
    """Tests ANA* when the path to the goal is completely blocked."""
    start, goal = (0, 0), (4, 0)
    
    result = get_final_result(anytime_nonparametric_astar(sample_grid, start, goal, heuristic))
    
    assert result is not None
    assert result["path"] is None
    assert result["cost"] == INF
    assert result["is_final"] == True

def test_complex_path_optimal(sample_grid):
    """Tests ANA* on a complex path; it should find the optimal path."""
    start, goal = (0, 0), (4, 4)
    
    result = get_final_result(anytime_nonparametric_astar(sample_grid, start, goal, heuristic))
    
    assert result is not None
    expected_path = [(0, 0), (1, 0), (2, 0), (2, 1), (3, 1), (3, 2), (4, 2), (4, 3), (4, 4)]
    assert result["path"] == expected_path
    assert result["cost"] == 12
    assert result["path"][0] == start
    assert result["path"][-1] == goal

def test_ana_finds_improving_solutions(trap_grid):
    """
    Tests the core "anytime" feature of ANA*.
    On the trap_grid, it may find a suboptimal path first, then should
    continue to find the optimal path.
    """
    start, goal = (0, 3), (4, 3)
    
    solutions, final_sol = get_all_solutions(anytime_nonparametric_astar(trap_grid, start, goal, heuristic))
    
    # --- Check Solutions ---
    # ANA* is not guaranteed to find the 20-cost path first, unlike a heavily weighted AWA*.
    # It depends on the order of expansion. The key is that costs must be non-increasing.
    assert len(solutions) > 0, "ANA* should have found at least one solution."
    
    costs = [sol["cost"] for sol in solutions]
    assert all(costs[i] >= costs[i+1] for i in range(len(costs)-1)), "Solution costs should not increase."

    first_sol = solutions[0]
    
    # --- Check Final Solution (Optimal) ---
    assert final_sol is not None
    assert final_sol["cost"] == 10
    assert final_sol["path"] == [
        (0, 3), (0, 2), (0, 1), (0, 0), 
        (1, 0), (2, 0), (3, 0), (4, 0), 
        (4, 1), (4, 2), (4, 3)
    ]
    
    # The final cost must be better than or equal to the first
    assert final_sol["cost"] <= first_sol["cost"]
