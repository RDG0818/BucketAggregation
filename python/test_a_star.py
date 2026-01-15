import pytest
from a_star import astar

@pytest.fixture
def sample_grid():
    return [
        [1, 1, 1, 0, 1], # 0
        [1, 5, 5, 1, 1], # 1
        [1, 4, 3, 1, 1], # 2
        [0, 2, 1, 5, 0], # 3
        [1, 0, 1, 1, 1]  # 4
    ] # 0  1  2  3  4

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def test_simple_path(sample_grid):
    start, goal = (0, 0), (0, 2)
    result = next(astar(sample_grid, start, goal, heuristic))
    assert result["path"] == [(0, 0), (0, 1), (0, 2)]
    assert result["cost"] == 2

def test_no_path(sample_grid):
    start, goal = (0, 0), (0, 3)
    result = next(astar(sample_grid, start, goal, heuristic))
    assert result["path"] is None

def test_weighted_path(sample_grid):
    start, goal = (0, 0), (2, 2)
    result = next(astar(sample_grid, start, goal, heuristic))
    assert result["path"] == [(0, 0), (1, 0), (2, 0), (2, 1), (2, 2)]
    assert result["cost"] == 9