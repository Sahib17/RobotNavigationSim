import math
import numpy as np
from collections import deque


def evaluate_path_density(grid, start, end, obstacle_weight=10):
    """
    Pathfinding algorithm that finds a path while considering obstacle density.
    Prefers paths with fewer obstacles nearby.

    Args:
        grid: 2D numpy array, True represents obstacles
        start: Tuple (y, x) of start position in grid coordinates
        end: Tuple (y, x) of end position in grid coordinates
        obstacle_weight: Weight factor for obstacles in density calculation

    Returns:
        List of grid cells [(y1, x1), (y2, x2), ...] forming the path from start to end,
        or None if no path found
    """
    # Directions for movement (8-connected grid)
    directions = [
        (0, 1), (1, 0), (0, -1), (-1, 0),  # Cardinals
        (1, 1), (-1, 1), (1, -1), (-1, -1)  # Diagonals
    ]

    # Grid dimensions
    height, width = grid.shape

    # Create density grid (higher value = more obstacles nearby)
    density_grid = calculate_obstacle_density(grid, obstacle_weight)

    # Distance grid (distance from start)
    distance = {}
    distance[start] = 0

    # Total cost (distance + density penalty)
    cost = {}
    cost[start] = 0

    # Priority queue for Dijkstra
    queue = [(0, start)]
    import heapq

    # Dictionary to track path
    came_from = {}

    # Process queue
    while queue:
        _, current = heapq.heappop(queue)

        # Reached the goal
        if current == end:
            return reconstruct_path(came_from, current, start)

        # Check all neighbors
        for dy, dx in directions:
            neighbor = (current[0] + dy, current[1] + dx)

            # Skip invalid neighbors
            if (neighbor[0] < 0 or neighbor[0] >= height or
                    neighbor[1] < 0 or neighbor[1] >= width or
                    grid[neighbor[0], neighbor[1]]):
                continue

            # Check for diagonal movement through obstacles (corner cutting)
            if abs(dx) == 1 and abs(dy) == 1:
                # Check if the two adjacent cardinal cells are free
                if (grid[current[0] + dy, current[1]] or
                        grid[current[0], current[1] + dx]):
                    continue  # Can't cut corners diagonally through walls

            # Calculate move cost (diagonal costs more)
            if abs(dx) == 1 and abs(dy) == 1:
                move_cost = 1.414  # sqrt(2)
            else:
                move_cost = 1.0

            # Add density penalty
            density_penalty = density_grid[neighbor[0], neighbor[1]]

            # Calculate new distance and cost
            new_distance = distance[current] + move_cost
            new_cost = new_distance + density_penalty

            # If we haven't seen this position or if it's better than a previous path
            if neighbor not in cost or new_cost < cost[neighbor]:
                distance[neighbor] = new_distance
                cost[neighbor] = new_cost
                came_from[neighbor] = current

                # Add to queue
                heapq.heappush(queue, (new_cost, neighbor))

    # No path found
    return None


def calculate_obstacle_density(grid, weight=10, radius=3):
    """
    Calculate obstacle density for each cell in the grid.
    A higher value means more obstacles nearby.

    Args:
        grid: 2D numpy array, True represents obstacles
        weight: Weight factor for obstacles
        radius: Radius to check for obstacles

    Returns:
        2D numpy array with density values
    """
    height, width = grid.shape
    density = np.zeros_like(grid, dtype=float)

    # For each cell in the grid
    for y in range(height):
        for x in range(width):
            # Skip obstacles (they'll have infinite density)
            if grid[y, x]:
                density[y, x] = float('inf')
                continue

            # Count obstacles in radius
            obstacle_count = 0
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    ny, nx = y + dy, x + dx

                    # Check bounds
                    if not (0 <= ny < height and 0 <= nx < width):
                        continue

                    # Calculate distance
                    distance = math.sqrt(dx ** 2 + dy ** 2)
                    if distance > radius:
                        continue

                    # Add to density if obstacle (weighted by distance)
                    if grid[ny, nx]:
                        # Closer obstacles have higher weight
                        obstacle_count += weight * (1 - distance / radius)

            density[y, x] = obstacle_count

    return density


def sonar_guided_path(grid, start, end, heuristic_func):
    """
    A* pathfinding that simulates sonar-like obstacle detection.
    Uses 360° "view" to evaluate and choose paths with fewer obstacles.

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

    # Estimate total cost (g + h + obstacle_factor)
    f_score = {start: heuristic_func(start, end)}

    # Add start to open set
    import heapq
    heapq.heappush(open_set, (f_score[start], start))

    # Grid dimensions
    height, width = grid.shape

    # Pre-compute obstacle density map
    obstacle_density = calculate_obstacle_density(grid, weight=8, radius=4)

    while open_set:
        # Get node with lowest f_score
        _, current = heapq.heappop(open_set)

        # If reached the goal
        if current == end:
            return reconstruct_path(came_from, current, start)

        # Mark as processed
        closed_set.add(current)

        # Check each neighbor
        for dy, dx in directions:
            neighbor = (current[0] + dy, current[1] + dx)

            # Skip invalid neighbors
            if (neighbor[0] < 0 or neighbor[0] >= height or
                    neighbor[1] < 0 or neighbor[1] >= width or
                    grid[neighbor[0], neighbor[1]] or
                    neighbor in closed_set):
                continue

            # Check for diagonal movement through obstacles (corner cutting)
            if abs(dx) == 1 and abs(dy) == 1:
                # Check if the two adjacent cardinal cells are free
                if (grid[current[0] + dy, current[1]] or
                        grid[current[0], current[1] + dx]):
                    continue  # Can't cut corners diagonally through walls

            # Calculate g_score for this neighbor
            # Diagonal movement costs more
            if abs(dx) == 1 and abs(dy) == 1:
                move_cost = 1.414  # sqrt(2)
            else:
                move_cost = 1.0

            tentative_g = g_score[current] + move_cost

            # Include obstacle density penalty
            density_penalty = min(10, obstacle_density[neighbor[0], neighbor[1]]) / 5

            # If we already have a better path to this neighbor, skip
            if neighbor in g_score and tentative_g >= g_score[neighbor]:
                continue

            # This path is better, record it
            came_from[neighbor] = current
            g_score[neighbor] = tentative_g

            # Calculate f_score including heuristic and density penalty
            h_value = heuristic_func(neighbor, end)
            f_score[neighbor] = tentative_g + h_value + density_penalty

            # Add to open set if not already there
            if not any(neighbor == item[1] for item in open_set):
                heapq.heappush(open_set, (f_score[neighbor], neighbor))

    # No path found
    return None


def simulate_sonar(grid, position, heading, ray_count=16, sensor_range=20):
    """
    Simulate sonar-like obstacle detection.

    Args:
        grid: 2D numpy array, True represents obstacles
        position: Tuple (y, x) of current position
        heading: Direction in radians
        ray_count: Number of rays to cast
        sensor_range: Range of detection

    Returns:
        List of (distance, angle) tuples for detected obstacles
    """
    height, width = grid.shape
    detections = []

    # Cast rays in 360 degrees
    angle_span = 2 * math.pi
    start_angle = heading - math.pi

    for i in range(ray_count):
        angle = start_angle + i * (angle_span / ray_count)

        # Cast ray
        for distance in range(1, sensor_range + 1):
            y = int(position[0] + distance * math.sin(angle))
            x = int(position[1] + distance * math.cos(angle))

            # Check bounds
            if not (0 <= y < height and 0 <= x < width):
                break

            # Check for obstacle
            if grid[y, x]:
                detections.append((distance, angle))
                break

    return detections


def vector_field_navigation(grid, start, end):
    """
    Navigation using vector fields to find paths that stay away from obstacles.

    Args:
        grid: 2D numpy array, True represents obstacles
        start: Tuple (y, x) of start position in grid coordinates
        end: Tuple (y, x) of end position in grid coordinates

    Returns:
        List of grid cells [(y1, x1), (y2, x2), ...] forming the path from start to end,
        or None if no path found
    """
    # Grid dimensions
    height, width = grid.shape

    # Create distance field from goal (using BFS)
    distance_field = np.full((height, width), -1, dtype=int)
    distance_field[end[0], end[1]] = 0

    # BFS queue
    queue = deque([end])

    # Directions for movement (4-connected grid)
    directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    # Fill distance field
    while queue:
        y, x = queue.popleft()
        current_dist = distance_field[y, x]

        for dy, dx in directions:
            ny, nx = y + dy, x + dx

            # Check bounds and obstacles
            if not (0 <= ny < height and 0 <= nx < width) or grid[ny, nx]:
                continue

            # If not visited yet
            if distance_field[ny, nx] == -1:
                distance_field[ny, nx] = current_dist + 1
                queue.append((ny, nx))

    # If start is unreachable
    if distance_field[start[0], start[1]] == -1:
        return None

    # Create repulsion field from obstacles
    repulsion_field = np.zeros((height, width, 2), dtype=float)  # (dy, dx) vectors

    # For each cell, calculate repulsion from nearby obstacles
    for y in range(height):
        for x in range(width):
            # Skip obstacles
            if grid[y, x]:
                continue

            # Check nearby obstacles
            for dy in range(-3, 4):
                for dx in range(-3, 4):
                    ny, nx = y + dy, x + dx

                    # Check bounds
                    if not (0 <= ny < height and 0 <= nx < width):
                        continue

                    # If obstacle, add repulsion
                    if grid[ny, nx]:
                        # Distance to obstacle
                        dist = math.sqrt(dy ** 2 + dx ** 2)
                        if dist < 0.1:  # Avoid division by zero
                            dist = 0.1

                        # Repulsion force (inversely proportional to distance)
                        force = 1.0 / dist

                        # Direction away from obstacle
                        direction_y = -dy / dist
                        direction_x = -dx / dist

                        # Add to repulsion field
                        repulsion_field[y, x, 0] += direction_y * force
                        repulsion_field[y, x, 1] += direction_x * force

    # Follow gradient of distance field + repulsion field
    path = [start]
    current = start

    # Maximum steps to avoid infinite loops
    max_steps = height * width
    steps = 0

    while current != end and steps < max_steps:
        y, x = current
        steps += 1

        best_neighbor = None
        best_score = float('inf')

        for dy, dx in directions:
            ny, nx = y + dy, x + dx

            # Check bounds and obstacles
            if not (0 <= ny < height and 0 <= nx < width) or grid[ny, nx]:
                continue

            # Can't go backward
            if distance_field[ny, nx] == -1:
                continue

            # Combine distance gradient and repulsion
            # Lower distance to goal is better
            distance_score = distance_field[ny, nx]

            # Higher repulsion is worse
            repulsion_y, repulsion_x = repulsion_field[ny, nx]
            repulsion_score = math.sqrt(repulsion_y ** 2 + repulsion_x ** 2)

            # Total score (lower is better)
            total_score = distance_score + repulsion_score * 0.5

            if total_score < best_score:
                best_score = total_score
                best_neighbor = (ny, nx)

        if best_neighbor is None:
            # Stuck - no good neighbors
            return None

        # Move to best neighbor
        current = best_neighbor
        path.append(current)

    return path if steps < max_steps else None


def reconstruct_path(came_from, current, start):
    """Helper function to reconstruct path from came_from dict."""
    path = []
    while current in came_from:
        path.append(current)
        current = came_from[current]
    path.append(start)
    path.reverse()
    return path