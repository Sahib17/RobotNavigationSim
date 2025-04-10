import pygame
import numpy as np
import math


class MappingSystem:
    """System for mapping out an environment as the robot explores it."""

    def __init__(self, screen_width, screen_height):
        # Map dimensions
        self.screen_width = screen_width
        self.screen_height = screen_height

        # Create an array to track which parts of the map have been seen
        # 0 = not seen, 1 = seen (explored)
        self.visibility_map = np.zeros((screen_height, screen_width), dtype=float)

        # Create a surface to draw the fog of war (unexplored areas)
        self.fog_surface = pygame.Surface((screen_width, screen_height), pygame.SRCALPHA)
        self.fog_surface.fill((0, 0, 0, 255))  # Start with completely black (unexplored)

        # Keep track of all positions where sonar beams have detected walls
        self.detected_walls = set()  # (x, y) pairs

        # Settings for debug/testing
        self.robot_speed_multiplier = 1.0  # Default speed
        self.sonar_range_multiplier = 1.0  # Default range
        self.fog_alpha = 220  # 0 (transparent) to 255 (opaque)

        # Real-time debug settings
        self.show_debug_info = False
        self.show_full_map = False  # Debug option to see the whole map

    def update_visibility_from_sonar(self, robot_position, sonar_detections, sonar_range):
        """Update the visibility map based on sonar detections."""
        if robot_position is None:
            return

        robot_x, robot_y = int(robot_position[0]), int(robot_position[1])

        # Mark area around robot as visible
        visible_radius = int(sonar_range * 0.6)
        self._update_visibility_circle(robot_x, robot_y, visible_radius)

        # Add visibility from sonar beams
        for detection in sonar_detections:
            if len(detection) >= 3:  # If detection has enough data (x, y, distance)
                x, y, dist = detection[0], detection[1], abs(detection[2])
                x, y = int(x), int(y)

                if 0 <= x < self.screen_width and 0 <= y < self.screen_height:
                    # Add wall detection
                    if dist > 0:  # An actual wall detection, not max range
                        self.detected_walls.add((x, y))

                    # Update visibility along the beam
                    self._update_visibility_along_line(robot_x, robot_y, x, y)

    def _update_visibility_circle(self, center_x, center_y, radius):
        """Mark a circular area as visible."""
        for y in range(max(0, center_y - radius), min(self.screen_height, center_y + radius + 1)):
            for x in range(max(0, center_x - radius), min(self.screen_width, center_x + radius + 1)):
                # Check if the point is within the circle
                if (x - center_x) ** 2 + (y - center_y) ** 2 <= radius ** 2:
                    if 0 <= y < self.visibility_map.shape[0] and 0 <= x < self.visibility_map.shape[1]:
                        self.visibility_map[y, x] = 1.0

    def _update_visibility_along_line(self, x1, y1, x2, y2):
        """Mark a line as visible (for sonar beam)."""
        # Bresenham's line algorithm to ensure all pixels in the line are marked
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        sx = 1 if x1 < x2 else -1
        sy = 1 if y1 < y2 else -1
        err = dx - dy

        x, y = x1, y1
        while True:
            if 0 <= y < self.visibility_map.shape[0] and 0 <= x < self.visibility_map.shape[1]:
                self.visibility_map[y, x] = 1.0

            if x == x2 and y == y2:
                break

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    def update_fog_surface(self):
        """Update the fog surface based on current visibility."""
        self.fog_surface.fill((0, 0, 0, 0))  # Clear the surface

        # Option to show full map (debug)
        if self.show_full_map:
            return

        # Create a pixel array for faster manipulation
        fog_array = pygame.surfarray.pixels_alpha(self.fog_surface)

        # Set alpha based on visibility map
        for y in range(self.screen_height):
            for x in range(self.screen_width):
                if y < self.visibility_map.shape[0] and x < self.visibility_map.shape[1]:
                    if self.visibility_map[y, x] == 0:
                        fog_array[x, y] = self.fog_alpha

        # Release the pixel array
        del fog_array

    def draw(self, screen, original_map):
        """Draw the visibility effects onto the screen."""
        # Draw only the detected walls (optional - for creating a "built map")
        # This would replace drawing the original map to simulate the robot building its own map
        # self._draw_detected_walls(screen)

        # Apply fog of war effect over the map
        self.update_fog_surface()
        screen.blit(self.fog_surface, (0, 0))

        # Draw debug information
        if self.show_debug_info:
            self._draw_debug_info(screen)

    def _draw_detected_walls(self, screen):
        """Draw only the walls that have been detected by the robot."""
        for x, y in self.detected_walls:
            pygame.draw.rect(screen, (50, 50, 50), (x, y, 1, 1))

    def _draw_debug_info(self, screen):
        """Draw debug information on screen."""
        font = pygame.font.SysFont(None, 24)

        # Show robot speed multiplier
        speed_text = font.render(f"Speed: x{self.robot_speed_multiplier:.1f}", True, (0, 255, 0))
        screen.blit(speed_text, (10, self.screen_height - 70))

        # Show sonar range multiplier
        range_text = font.render(f"Sonar range: x{self.sonar_range_multiplier:.1f}", True, (0, 255, 0))
        screen.blit(range_text, (10, self.screen_height - 45))

        # Show map visibility percentage
        visible_pct = np.sum(self.visibility_map) / (self.visibility_map.shape[0] * self.visibility_map.shape[1]) * 100
        vis_text = font.render(f"Mapped: {visible_pct:.1f}%", True, (0, 255, 0))
        screen.blit(vis_text, (10, self.screen_height - 20))

    def toggle_debug_info(self):
        """Toggle debug information display."""
        self.show_debug_info = not self.show_debug_info

    def toggle_full_map(self):
        """Toggle showing the full map (debug option)."""
        self.show_full_map = not self.show_full_map

    def increase_robot_speed(self):
        """Increase the robot speed multiplier."""
        self.robot_speed_multiplier = min(5.0, self.robot_speed_multiplier + 0.5)
        return self.robot_speed_multiplier

    def decrease_robot_speed(self):
        """Decrease the robot speed multiplier."""
        self.robot_speed_multiplier = max(0.5, self.robot_speed_multiplier - 0.5)
        return self.robot_speed_multiplier

    def increase_sonar_range(self):
        """Increase the sonar range multiplier."""
        self.sonar_range_multiplier = min(3.0, self.sonar_range_multiplier + 0.5)
        return self.sonar_range_multiplier

    def decrease_sonar_range(self):
        """Decrease the sonar range multiplier."""
        self.sonar_range_multiplier = max(0.5, self.sonar_range_multiplier - 0.5)
        return self.sonar_range_multiplier

    def increase_fog_transparency(self):
        """Make fog more transparent."""
        self.fog_alpha = max(50, self.fog_alpha - 25)
        return self.fog_alpha

    def decrease_fog_transparency(self):
        """Make fog less transparent."""
        self.fog_alpha = min(255, self.fog_alpha + 25)
        return self.fog_alpha

    def reset(self):
        """Reset the mapping system."""
        self.visibility_map = np.zeros((self.screen_height, self.screen_width), dtype=float)
        self.detected_walls = set()
        self.fog_surface.fill((0, 0, 0, 255))