import heapq
import sys

INF = sys.maxsize

# type aliasing
Node = tuple[int, int]
Grid = list[list[int]]

def anytime_weighted_astar(grid: Grid, start: Node, goal: Node, heuristic, w: float = 2.0):
    """Anytime Weighted A* search algorithm implementation."""

    if w < 1.0:
        w = 1.0

    rows = len(grid)
    cols = len(grid[0]) if rows > 0 else 0
    obstacle_value = 0

    def _is_valid(row: int, col: int) -> bool:
        return 0 <= row < rows and 0 <= col < cols

    def _is_walkable(row: int, col: int) -> bool:
        return _is_valid(row, col) and grid[row][col] != obstacle_value

    def get_neighbors(node: Node) -> list[tuple[Node, int]]:
        r, c = node
        neighbors = []
        for dr, dc in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nr, nc = r + dr, c + dc
            if _is_walkable(nr, nc):
                move_cost = grid[nr][nc]
                neighbors.append(((nr, nc), move_cost))
        return neighbors

    def reconstruct_path(came_from: dict, current: Node) -> list[Node]:
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        return path[::-1]

    if not _is_valid(start[0], start[1]) or not _is_walkable(start[0], start[1]):
        yield {"path": None, "cost": INF, "expanded_nodes": 0, "is_final": True}
        return

    if not _is_valid(goal[0], goal[1]) or not _is_walkable(goal[0], goal[1]):
        yield {"path": None, "cost": INF, "expanded_nodes": 0, "is_final": True}
        return

    open_set = []
    g_score = {start: 0}
    came_from = {}
    closed_set = set()

    incumbent_path = None
    incumbent_cost = INF

    expanded_nodes = 0

    start_h = heuristic(start, goal)
    
    heapq.heappush(open_set, (start_h * w, start_h, start))

    while open_set:
        _, h, current = heapq.heappop(open_set)

        if g_score[current] + h >= incumbent_cost:
            continue

        if current in closed_set:
            continue

        closed_set.add(current)
        expanded_nodes += 1

        if current == goal:
            path_cost = g_score[current]

            if path_cost < incumbent_cost:
                incumbent_cost = path_cost
                incumbent_path = reconstruct_path(came_from, current)

                yield {
                    "path": incumbent_path,
                    "cost": incumbent_cost,
                    "expanded_nodes": expanded_nodes,
                    "is_final": False,
                    "new_solution_found": True
                }

            continue

        current_g_score = g_score[current]
        for neighbor, move_cost in get_neighbors(current):
            tentative_g_score = current_g_score + move_cost

            if tentative_g_score < g_score.get(neighbor, INF):
                neighbor_h = heuristic(neighbor, goal)
                if tentative_g_score + neighbor_h >= incumbent_cost:
                    continue

                came_from[neighbor] = current
                g_score[neighbor] = tentative_g_score
                
                heapq.heappush(open_set, (tentative_g_score + neighbor_h * w, neighbor_h, neighbor))

                if neighbor in closed_set:
                    closed_set.remove(neighbor)

    yield {
        "path": incumbent_path,
        "cost": incumbent_cost,
        "expanded_nodes": expanded_nodes,
        "is_final": True,
        "new_solution_found": False
    }
    return
