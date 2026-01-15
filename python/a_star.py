import heapq

# type aliasing
Node = tuple[int, int]
Grid = list[list[int]]

def astar(grid: Grid, start: Node, goal: Node, heuristic):
    """
    A* search algorithm implementation.

    :param grid: A 2D array where 0 represents an obstacle and a positive integer is the cost.
    :param start: The starting node.
    :param goal: The goal node.
    :param heuristic: A function that takes two nodes and returns the estimated cost to get from the first node to the second.
    :return: A tuple containing the path from the start to the goal and the cost of the path.
    """
    def get_neighbors(node: Node):
        x, y = node
        neighbors = []
        for dx, dy in [(0, 1), (0, -1), (1, 0), (-1, 0)]:
            nx, ny = x + dx, y + dy
            if 0 <= nx < len(grid) and 0 <= ny < len(grid[0]) and grid[nx][ny] != 0:
                neighbors.append((nx, ny))
        return neighbors

    frontier = [(0, start)]
    came_from = {}
    cost_so_far = {start: 0}
    expanded_nodes = 0

    while frontier:
        _, current = heapq.heappop(frontier)
        expanded_nodes += 1

        if current == goal:
            break

        for next_node in get_neighbors(current):
            new_cost = cost_so_far[current] + grid[next_node[0]][next_node[1]]
            if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                cost_so_far[next_node] = new_cost
                priority = new_cost + heuristic(goal, next_node)
                heapq.heappush(frontier, (priority, next_node))
                came_from[next_node] = current

    path = []
    if goal in came_from:
        current = goal
        while current != start:
            path.append(current)
            current = came_from[current]
        path.append(start)
        path.reverse()

    yield {
        "path": path if path else None,
        "cost": cost_so_far.get(goal, -1),
        "expanded_nodes": expanded_nodes,
        "is_final": True
    }
