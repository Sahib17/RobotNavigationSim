import heapq
from collections import deque


def astar(grid, start, end, heuristic_func):
    """
    A* pathfinding algorithm.

    Args:
        grid: 2D numpy array, True represents obstacles
        start: Tuple (y, x) of start position in grid coordinates
        end: Tuple (y, x) of end position in grid coordinates
        heuristic_func: Function to calculate heuristic distance

    Returns:
        List of grid cells [(y1, x1), (y2, x2), ...] forming the path from start to end,
        or None if no path found
    """
    # Directions for movement (8-connected grid)
    directions = [
        (0, 1), (1, 0), (0, -1), (-1, 0),  # Cardinals
        (1, 1), (-1, 1), (1, -1), (-1, -1)  # Diagonals
    ]

    # Initialize open and closed sets
    open_set = []
    closed_set = set()

    # Dictionary to track path
    came_from = {}

    # Distance from start
    g_score = {start: 0}

    # Estimated total cost (g + h)
    f_score = {start: heuristic_func(start, end)}

    # Add start to open set
    heapq.heappush(open_set, (f_score[start], start))

    # Grid dimensions
    height, width = grid.shape

    while open_set:
        # Get node with lowest f_score
        _, current = heapq.heappop(open_set)

        # If reached the goal
        if current == end:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        # Mark as processed
        closed_set.add(current)

        # Check each neighbor
        for dy, dx in directions:
            neighbor = (current[0] + dy, current[1] + dx)

            # Check if valid position
            if not (0 <= neighbor[0] < height and 0 <= neighbor[1] < width):
                continue

            # Check if obstacle
            if grid[neighbor[0], neighbor[1]]:
                continue

            # Check if already processed
            if neighbor in closed_set:
                continue

            # Check for diagonal movement through walls (corner cutting)
            if abs(dx) == 1 and abs(dy) == 1:
                # Check if the two adjacent cardinal cells are free
                if (grid[current[0] + dy, current[1]] or
                        grid[current[0], current[1] + dx]):
                    continue  # Can't cut corners diagonally through walls

            # Calculate tentative g_score
            # Diagonal movement costs more
            if abs(dx) == 1 and abs(dy) == 1:
                tentative_g = g_score[current] + 1.414  # sqrt(2)
            else:
                tentative_g = g_score[current] + 1

            # If we already have a better path to this neighbor, skip
            if neighbor in g_score and tentative_g >= g_score[neighbor]:
                continue

            # This path is better, record it
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            f_score[neighbor] = tentative_g + heuristic_func(neighbor, end)

            # Add to open set if not already there
            if not any(neighbor == item[1] for item in open_set):
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    # No path found
    return None


def dijkstra(grid, start, end):
    """
    Dijkstra's algorithm for pathfinding.

    Args:
        grid: 2D numpy array, True represents obstacles
        start: Tuple (y, x) of start position in grid coordinates
        end: Tuple (y, x) of end position in grid coordinates

    Returns:
        List of grid cells [(y1, x1), (y2, x2), ...] forming the path from start to end,
        or None if no path found
    """
    # Directions for movement (4-connected grid for simplicity)
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Cardinals only

    # Initialize priority queue
    queue = [(0, start)]

    # Dictionary to track path
    came_from = {}

    # Distance from start
    cost_so_far = {start: 0}

    # Grid dimensions
    height, width = grid.shape

    while queue:
        # Get node with lowest cost
        current_cost, current = heapq.heappop(queue)

        # If reached the goal
        if current == end:
            # Reconstruct path
            path = []
            while current in came_from:
                path.append(current)
                current = came_from[current]
            path.append(start)
            path.reverse()
            return path

        # Check each neighbor
        for dy, dx in directions:
            neighbor = (current[0] + dy, current[1] + dx)

            # Check if valid position
            if not (0 <= neighbor[0] < height and 0 <= neighbor[1] < width):
                continue

            # Check if obstacle
            if grid[neighbor[0], neighbor[1]]:
                continue

            # All moves cost 1 in Dijkstra
            new_cost = cost_so_far[current] + 1

            # If we haven't seen this position before, or we found a better path
            if neighbor not in cost_so_far or new_cost < cost_so_far[neighbor]:
                cost_so_far[neighbor] = new_cost
                priority = new_cost
                heapq.heappush(queue, (priority, neighbor))
                came_from[neighbor] = current

    # No path found
    return None


def grassfire(grid, start, end):
    """
    Grassfire/wavefront algorithm for pathfinding.
    Similar to breadth-first search, expanding outward from the goal.

    Args:
        grid: 2D numpy array, True represents obstacles
        start: Tuple (y, x) of start position in grid coordinates
        end: Tuple (y, x) of end position in grid coordinates

    Returns:
        List of grid cells [(y1, x1), (y2, x2), ...] forming the path from start to end,
        or None if no path found
    """
    # Directions for movement (4-connected grid)
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Cardinals only

    # Grid dimensions
    height, width = grid.shape

    # Distance grid (wavefront)
    # -1 = unvisited, -2 = obstacle
    distance_grid = [[-1 for _ in range(width)] for _ in range(height)]

    # Mark obstacles
    for y in range(height):
        for x in range(width):
            if grid[y, x]:
                distance_grid[y][x] = -2

    # Queue for BFS
    queue = deque([end])

    # Start distance from end is 0
    distance_grid[end[0]][end[1]] = 0

    # BFS to fill the distance grid
    while queue:
        current = queue.popleft()
        current_dist = distance_grid[current[0]][current[1]]

        # Check each neighbor
        for dy, dx in directions:
            ny, nx = current[0] + dy, current[1] + dx

            # Check if valid position
            if not (0 <= ny < height and 0 <= nx < width):
                continue

            # If unvisited and not an obstacle
            if distance_grid[ny][nx] == -1:
                distance_grid[ny][nx] = current_dist + 1
                queue.append((ny, nx))

    # If start is unreachable
    if distance_grid[start[0]][start[1]] == -1:
        return None

    # Trace path from start to end by always moving to the neighbor with lowest distance
    path = [start]
    current = start

    while current != end:
        best_dist = float('inf')
        best_next = None

        # Find neighbor with lowest distance
        for dy, dx in directions:
            ny, nx = current[0] + dy, current[1] + dx

            # Check if valid position
            if not (0 <= ny < height and 0 <= nx < width):
                continue

            # If this neighbor has a valid distance and is better than current best
            if distance_grid[ny][nx] >= 0 and distance_grid[ny][nx] < best_dist:
                best_dist = distance_grid[ny][nx]
                best_next = (ny, nx)

        # If we can't find a better neighbor, path is invalid
        if best_next is None:
            return None

        # Move to best neighbor
        current = best_next
        path.append(current)

    return path


# Utility function to print a path on grid (for debugging)
def print_path_on_grid(grid, path):
    """
    Prints a visualization of the path on the grid for debugging.

    Args:
        grid: 2D numpy array, True represents obstacles
        path: List of (y, x) tuples representing the path
    """
    height, width = grid.shape

    # Create a visualization grid
    viz_grid = []
    for y in range(height):
        row = []
        for x in range(width):
            if grid[y, x]:
                row.append("#")  # Obstacle
            else:
                row.append(".")  # Free space
        viz_grid.append(row)

    # Mark the path
    for i, (y, x) in enumerate(path):
        if i == 0:
            viz_grid[y][x] = "S"  # Start
        elif i == len(path) - 1:
            viz_grid[y][x] = "E"  # End
        else:
            viz_grid[y][x] = "o"  # Path

    # Print the grid
    for row in viz_grid:
        print("".join(row))