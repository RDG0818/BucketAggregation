import math
from ana_bucket_heap import ANABucketHeap, SearchNode

INF = math.inf

# type aliasing
Node = tuple[int, int]
Grid = list[list[int]]

def anytime_nonparametric_astar(grid: Grid, start: Node, goal: Node, heuristic):
    """Anytime Non-parametric A* (ANA*) search algorithm using a BucketHeap."""

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

    # Use the ANABucketHeap for the open set
    open_set = ANABucketHeap()
    
    g_score = {start: 0}
    came_from = {}
    
    expanded_nodes = 0
    
    start_h = heuristic(start, goal)
    start_node = SearchNode(start, g_score=0, h_score=start_h)
    open_set.push(start_node)

    while len(open_set) > 0:
        # Pop the node with the highest potential from the bucket heap
        current_node = open_set.pop()
        if current_node is None: continue
        
        current_state = current_node.state

        # Pruning step: if the best-case path from this node (f_score) is already
        # worse than our best solution, we discard it.
        if current_node.f_score >= open_set.incumbent_cost:
            continue

        expanded_nodes += 1

        if current_state == goal:
            path_cost = g_score[current_state]
            
            if path_cost < open_set.incumbent_cost:
                # Update the incumbent cost in the bucket heap, which reorders it
                open_set.update_incumbent(path_cost)
                incumbent_path = reconstruct_path(came_from, current_state)

                yield {
                    "path": incumbent_path,
                    "cost": path_cost,
                    "expanded_nodes": expanded_nodes,
                    "is_final": False,
                    "new_solution_found": True
                }
            
            continue

        current_g_score = g_score[current_state]
        for neighbor_state, move_cost in get_neighbors(current_state):
            tentative_g_score = current_g_score + move_cost

            if tentative_g_score < g_score.get(neighbor_state, INF):
                neighbor_h = heuristic(neighbor_state, goal)
                
                # Pruning step for successors
                if tentative_g_score + neighbor_h >= open_set.incumbent_cost:
                    continue

                came_from[neighbor_state] = current_state
                g_score[neighbor_state] = tentative_g_score
                
                neighbor_node = SearchNode(neighbor_state, g_score=tentative_g_score, h_score=neighbor_h)
                open_set.push(neighbor_node)

    # After the loop, find the final best path from the last known incumbent cost
    final_cost = open_set.incumbent_cost
    final_path = None
    if final_cost != INF:
        # We need to reconstruct the path that corresponds to this cost.
        # The 'came_from' dict for the goal state should be correct.
        final_path = reconstruct_path(came_from, goal)


    yield {
        "path": final_path,
        "cost": final_cost if final_path else INF,
        "expanded_nodes": expanded_nodes,
        "is_final": True,
        "new_solution_found": False
    }
    return
