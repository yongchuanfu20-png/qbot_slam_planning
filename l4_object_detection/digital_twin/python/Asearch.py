import numpy as np
import heapq

def compute_c_space(grid, robot_radius):
    """Performs binary dilation to create C-Space."""
    rows, cols = grid.shape
    c_space = np.copy(grid)
    obstacles = np.argwhere(grid == 255)
    for r, c in obstacles:
        for dr in range(-robot_radius, robot_radius + 1):
            for dc in range(-robot_radius, robot_radius + 1):
                # Check Euclidean distance for circular robot [cite: 27, 29]
                if np.sqrt(dr ** 2 + dc ** 2) <= robot_radius:
                    nr, nc = r + dr, c + dc
                    if 0 <= nr < rows and 0 <= nc < cols:
                        c_space[nr, nc] = 255
    return c_space



def get_heuristic(n, goal, h_type):
    x1, y1 = n
    x2, y2 = goal
    if h_type == "manhattan":
        return abs(x1 - x2) + abs(y1 - y2)
    elif h_type == "euclidean":
        return np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
    return 0


def search(grid, start, goal, algorithm="dijkstra", h_type="manhattan"):
    rows, cols = grid.shape
    queue = []
    # Priority Queue stores: (f_score, (r, c))
    heapq.heappush(queue, (0, start))

    parent_map = {}
    g_costs = {start: 0}
    visited = set()
    nodes_expanded = 0

    while queue:
        _, current = heapq.heappop(queue)

        if current in visited: continue
        visited.add(current)
        nodes_expanded += 1

        if current == goal:
            # Reconstruct path
            path = []
            while current in parent_map:
                path.append(current)
                current = parent_map[current]
            path.append(start)
            return path[::-1], visited, nodes_expanded

        # 4-Connectivity Motion Model
        for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            neighbor = (current[0] + dr, current[1] + dc)

            if 0 <= neighbor[0] < rows and 0 <= neighbor[1] < cols:
                if grid[neighbor] == 0:
                    new_g = g_costs[current] + 1

                    if neighbor not in g_costs or new_g < g_costs[neighbor]:
                        g_costs[neighbor] = new_g
                        h = get_heuristic(neighbor, goal, h_type) if algorithm == "a_star" else 0
                        f = new_g + h  # [cite: 45]
                        parent_map[neighbor] = current
                        heapq.heappush(queue, (f, neighbor))

    return [], visited, nodes_expanded