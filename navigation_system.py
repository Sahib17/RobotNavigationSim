import pygame
import numpy as np
import math
import heapq
from collections import deque

# Import pathfinding algorithms
from pathfinding_algorithms import astar, dijkstra, grassfire


class Navigator:
    """Handles grid creation, pathfinding and path management."""

    def __init__(self, grid_size=20):
        self.grid_size = grid_size
        self.grid = None
        self.grid_width = None
        self.grid_height = None
        self.obstacle_map = None
        self.path = []
        self.current_waypoint_index = 0
        self.screen_width = 800  # Default size
        self.screen_height = 600  # Default size
        self.algorithm = "astar"  # Default algorithm

    def set_algorithm(self, algorithm):
        """Set the pathfinding algorithm to use."""
        valid_algorithms = ["astar", "dijkstra", "grassfire"]
        if algorithm in valid_algorithms:
            self.algorithm = algorithm
            print(f"Using {algorithm.upper()} algorithm for navigation")
        else:
            print(f"Unknown algorithm: {algorithm}. Using A* instead.")
            self.algorithm = "astar"

    def initialize_grid(self, map_img, screen_width, screen_height):
        """Set up navigation grid based on map image."""
        self.screen_width = screen_width
        self.screen_height = screen_height
        self.grid_width = screen_width // self.grid_size
        self.grid_height = screen_height // self.grid_size
        self.grid = np.zeros((self.grid_height, self.grid_width), dtype=bool)

        print(f"Creating navigation grid of size {self.grid_height}x{self.grid_width}")

        # Create pixel-level obstacle map
        self.obstacle_map = np.zeros((screen_height, screen_width), dtype=bool)
        map_array = pygame.surfarray.array3d(map_img)

        # Mark obstacles at pixel level
        for y in range(screen_height):
            for x in range(screen_width):
                # Check if pixel is black/dark (wall)
                if x < map_array.shape[0] and y < map_array.shape[1] and np.all(map_array[x, y] < 50):
                    self.obstacle_map[y, x] = True

        # Create coarse grid for pathfinding
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                # Check multiple points in each cell to determine if it's an obstacle
                cell_has_obstacle = False
                for sy in range(3):
                    for sx in range(3):
                        px = min(screen_width - 1, x * self.grid_size + sx * (self.grid_size // 3))
                        py = min(screen_height - 1, y * self.grid_size + sy * (self.grid_size // 3))
                        if py < screen_height and px < screen_width and self.obstacle_map[py, px]:
                            cell_has_obstacle = True
                            break
                    if cell_has_obstacle:
                        break

                # Mark grid cell as obstacle
                if cell_has_obstacle:
                    self.grid[y, x] = True

        print("Grid initialization complete")

    def is_position_safe(self, pos, radius=15):
        """Check if a position is safe (not colliding with obstacles)."""
        if self.obstacle_map is None:
            return True

        # Check if position is valid
        if pos is None:
            return False

        # Check if center point is in bounds
        x, y = int(pos[0]), int(pos[1])
        if x < 0 or x >= self.screen_width or y < 0 or y >= self.screen_height:
            return False

        # Check if center point is in obstacle
        if self.obstacle_map[y, x]:
            return False

        # Check points around perimeter
        for angle in range(0, 360, 30):  # Check every 30 degrees
            rad_angle = math.radians(angle)
            px = int(x + radius * math.cos(rad_angle))
            py = int(y + radius * math.sin(rad_angle))

            # Check if point is out of bounds
            if (px < 0 or px >= self.screen_width or py < 0 or py >= self.screen_height):
                continue

            # Check if point is in obstacle
            if self.obstacle_map[py, px]:
                return False

        return True

    def heuristic(self, a, b):
        """Calculate Manhattan distance heuristic."""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def find_nearest_free_cell(self, cell):
        """Find nearest grid cell that's not an obstacle."""
        y, x = cell

        # Check increasing radius around cell
        for radius in range(1, 10):  # Maximum search radius
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    # Only check cells at the current radius
                    if abs(dx) == radius or abs(dy) == radius:
                        ny, nx = y + dy, x + dx
                        if (0 <= ny < self.grid_height and 0 <= nx < self.grid_width and
                                not self.grid[ny, nx]):
                            return (ny, nx)

        # If no free cell found within radius, return original
        return cell

    def find_path(self, start, end):
        """Find path using the selected algorithm."""
        # Convert pixel coordinates to grid coordinates
        start_grid = (min(int(start[1] // self.grid_size), self.grid_height - 1),
                      min(int(start[0] // self.grid_size), self.grid_width - 1))
        end_grid = (min(int(end[1] // self.grid_size), self.grid_height - 1),
                    min(int(end[0] // self.grid_size), self.grid_width - 1))

        # Ensure within grid bounds
        start_grid = (min(self.grid_height - 1, max(0, start_grid[0])),
                      min(self.grid_width - 1, max(0, start_grid[1])))
        end_grid = (min(self.grid_height - 1, max(0, end_grid[0])),
                    min(self.grid_width - 1, max(0, end_grid[1])))

        print(f"Finding path from grid {start_grid} to {end_grid} using {self.algorithm}")

        # Check if start or end is in a wall
        if self.grid[start_grid[0], start_grid[1]] or self.grid[end_grid[0], end_grid[1]]:
            # Try to find nearest free cell
            start_grid = self.find_nearest_free_cell(start_grid)
            end_grid = self.find_nearest_free_cell(end_grid)
            if self.grid[start_grid[0], start_grid[1]] or self.grid[end_grid[0], end_grid[1]]:
                print("No valid path available - start or end in obstacle")
                return None  # Still can't find valid start/end

        # Choose the appropriate algorithm
        path_grid = None
        if self.algorithm == "astar":
            path_grid = astar(self.grid, start_grid, end_grid, self.heuristic)
        elif self.algorithm == "dijkstra":
            path_grid = dijkstra(self.grid, start_grid, end_grid)
        elif self.algorithm == "grassfire":
            path_grid = grassfire(self.grid, start_grid, end_grid)
        else:
            # Default to A* if algorithm not recognized
            path_grid = astar(self.grid, start_grid, end_grid, self.heuristic)

        if path_grid:
            # Convert grid path to pixel coordinates
            path = []
            for cell in path_grid:
                # Calculate pixel coordinates (center of grid cell)
                px = cell[1] * self.grid_size + self.grid_size // 2
                py = cell[0] * self.grid_size + self.grid_size // 2
                path.append((px, py))

            smoothed_path = self.smooth_path(path)
            self.path = smoothed_path
            self.current_waypoint_index = 0
            print(f"Path found with {len(smoothed_path)} waypoints")
            return smoothed_path
        else:
            print("No path found")
            return None

    def smooth_path(self, path):
        """Add intermediate waypoints to smooth corners and improve navigation."""
        if len(path) < 3:
            return path

        smoothed = [path[0]]

        # Dynamic waypoint spacing based on environment complexity
        for i in range(1, len(path) - 1):
            prev_pt = path[i - 1]
            curr_pt = path[i]
            next_pt = path[i + 1]

            # Check if this is a corner point (change in direction)
            v1x = curr_pt[0] - prev_pt[0]
            v1y = curr_pt[1] - prev_pt[1]
            v2x = next_pt[0] - curr_pt[0]
            v2y = next_pt[1] - curr_pt[1]

            # Normalize vectors
            v1_len = max(0.01, math.sqrt(v1x ** 2 + v1y ** 2))
            v2_len = max(0.01, math.sqrt(v2x ** 2 + v2y ** 2))
            v1x, v1y = v1x / v1_len, v1y / v1_len
            v2x, v2y = v2x / v2_len, v2y / v2_len

            # Calculate dot product to find angle
            dot_product = v1x * v2x + v1y * v2y
            angle = math.acos(max(-1, min(1, dot_product)))

            # Check if the corner is in a tight area
            is_tight_area = False

            # Cast rays in all directions to check for nearby obstacles
            nearby_obstacles = 0
            for ray_angle in range(0, 360, 30):
                angle_rad = math.radians(ray_angle)
                for ray_dist in range(5, 80, 5):
                    ray_x = int(curr_pt[0] + ray_dist * math.cos(angle_rad))
                    ray_y = int(curr_pt[1] + ray_dist * math.sin(angle_rad))

                    if (ray_x < 0 or ray_x >= self.screen_width or
                            ray_y < 0 or ray_y >= self.screen_height):
                        break

                    if self.obstacle_map[ray_y, ray_x]:
                        if ray_dist < 40:  # If obstacle is close
                            nearby_obstacles += 1
                        break

            is_tight_area = nearby_obstacles >= 4  # If obstacles in multiple directions

            # If angle is significant (e.g., turning a corner), add intermediate points
            if abs(angle) > 0.3:  # About 17 degrees
                # For tight areas, add more intermediate points for smoother navigation
                waypoint_count = 3 if is_tight_area else 1

                # Add approaching points
                for j in range(1, waypoint_count + 1):
                    # Calculate a point approaching the corner
                    before_factor = 0.7 * (j / (waypoint_count + 1))
                    before = (
                        int(curr_pt[0] - v1x * self.grid_size * before_factor),
                        int(curr_pt[1] - v1y * self.grid_size * before_factor)
                    )
                    if self.is_position_safe(before):
                        smoothed.append(before)

                # Add the corner point
                if self.is_position_safe(curr_pt):
                    smoothed.append(curr_pt)

                # Add departing points
                for j in range(1, waypoint_count + 1):
                    # Calculate a point departing from the corner
                    after_factor = 0.7 * (j / (waypoint_count + 1))
                    after = (
                        int(curr_pt[0] + v2x * self.grid_size * after_factor),
                        int(curr_pt[1] + v2y * self.grid_size * after_factor)
                    )
                    if self.is_position_safe(after):
                        smoothed.append(after)
            else:
                # Just add the original point if not a corner
                if self.is_position_safe(curr_pt):
                    smoothed.append(curr_pt)

        # Add final point
        smoothed.append(path[-1])

        # Add extra waypoints for very long segments to ensure robot doesn't get stuck
        final_path = [smoothed[0]]

        for i in range(1, len(smoothed)):
            # Calculate distance between waypoints
            dist = math.sqrt((smoothed[i][0] - smoothed[i - 1][0]) ** 2 + (smoothed[i][1] - smoothed[i - 1][1]) ** 2)

            # If segment is very long, add intermediate waypoints
            if dist > self.grid_size * 5:  # If longer than 5 grid cells
                # Number of intermediate points to add
                num_intermediate = max(1, int(dist / (self.grid_size * 3)))

                for j in range(1, num_intermediate + 1):
                    # Calculate intermediate point
                    factor = j / (num_intermediate + 1)
                    inter_x = int(smoothed[i - 1][0] + (smoothed[i][0] - smoothed[i - 1][0]) * factor)
                    inter_y = int(smoothed[i - 1][1] + (smoothed[i][1] - smoothed[i - 1][1]) * factor)

                    if self.is_position_safe((inter_x, inter_y)):
                        final_path.append((inter_x, inter_y))

            final_path.append(smoothed[i])

        return final_path

    def get_current_waypoint(self):
        """Get the current waypoint the robot should move towards."""
        if not self.path or self.current_waypoint_index >= len(self.path):
            return None
        return self.path[self.current_waypoint_index]

    def advance_to_next_waypoint(self):
        """Move to the next waypoint in the path."""
        if self.path and self.current_waypoint_index < len(self.path) - 1:
            self.current_waypoint_index += 1
            print(f"Moving to waypoint {self.current_waypoint_index + 1}/{len(self.path)}")
            return True
        print("Reached final waypoint")
        return False

    def reset_path(self):
        """Clear the current path."""
        self.path = []
        self.current_waypoint_index = 0

    def is_path_complete(self):
        """Check if the path has been fully traversed."""
        return not self.path or self.current_waypoint_index >= len(self.path) - 1

    def draw_path(self, screen, current_color=(128, 0, 128), waypoint_color=(0, 255, 0),
                  line_color=(0, 255, 0), line_width=2, waypoint_size=3):
        """Draw the path on the screen."""
        if not self.path:
            return

        # Draw lines between waypoints
        for i in range(len(self.path) - 1):
            pygame.draw.line(screen, line_color, self.path[i], self.path[i + 1], line_width)

        # Draw waypoints
        for i, waypoint in enumerate(self.path):
            if i == self.current_waypoint_index:
                # Highlight current target waypoint
                pygame.draw.circle(screen, current_color, waypoint, waypoint_size + 2)
            else:
                # Regular waypoint
                pygame.draw.circle(screen, waypoint_color, waypoint, waypoint_size)

    def draw_grid(self, screen, color=(200, 200, 200, 100)):
        """Visualize the navigation grid for debugging."""
        grid_surface = pygame.Surface((self.screen_width, self.screen_height), pygame.SRCALPHA)

        # Draw grid cells
        for y in range(self.grid_height):
            for x in range(self.grid_width):
                rect = pygame.Rect(
                    x * self.grid_size,
                    y * self.grid_size,
                    self.grid_size,
                    self.grid_size
                )

                if self.grid[y, x]:
                    # Draw obstacle cells
                    pygame.draw.rect(grid_surface, (255, 0, 0, 100), rect)
                else:
                    # Draw grid lines
                    pygame.draw.rect(grid_surface, color, rect, 1)

        screen.blit(grid_surface, (0, 0))