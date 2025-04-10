import pygame
import sys
from pygame.locals import *
import math
import numpy as np
from robot_classes import Robot
from navigation_system import Navigator
from mapping_system import MappingSystem
from menu import Button

# Initialize pygame
pygame.init()

# Default map file - will be updated by the menu system
MAP_FILENAME = 'maps/cave.png'

# Default screen dimensions - will be updated based on the map image
SCREEN_WIDTH = 800
SCREEN_HEIGHT = 600

# Define colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
YELLOW = (255, 255, 0)
PURPLE = (128, 0, 128)


class RobotNavigationMapping:
    """Main application for robot navigation simulation with mapping capability."""

    def __init__(self):
        # Initialize screen
        self.screen = None
        self.map_img = None
        self.font = pygame.font.SysFont(None, 30)

        # Application state
        self.running = True
        self.setting_start = True  # First click sets start, second click sets end
        self.show_sonar = True
        self.mapping_enabled = True
        self.destination_reached = False  # New flag to track if destination is reached

        # Navigation elements
        self.start_pos = None
        self.end_pos = None
        self.robot = None
        self.navigator = Navigator(grid_size=20)

        # Load map and initialize screen
        self.load_map()

        # Initialize mapping system
        self.mapping_system = MappingSystem(self.navigator.screen_width, self.navigator.screen_height)

        # Set up clock for controlling frame rate
        self.clock = pygame.time.Clock()

    def load_map(self):
        """Load map image and set up screen."""
        try:
            # Load map image using the specified filename
            self.map_img = pygame.image.load(MAP_FILENAME)

            # Get dimensions from the image
            SCREEN_WIDTH = self.map_img.get_width()
            SCREEN_HEIGHT = self.map_img.get_height()
            print(f"Loaded image dimensions: {SCREEN_WIDTH}x{SCREEN_HEIGHT}")

            # Set up display
            self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
            pygame.display.set_caption('Robot Navigation with Mapping')

            # Initialize navigation grid
            self.navigator.initialize_grid(self.map_img, SCREEN_WIDTH, SCREEN_HEIGHT)

        except pygame.error as e:
            print(f"Error loading map {MAP_FILENAME}: {e}")
            sys.exit()

    def handle_events(self):
        """Process user input events."""
        for event in pygame.event.get():
            if event.type == QUIT:
                self.running = False

            elif event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    self.running = False

                elif event.key == K_r:
                    self.reset_simulation()

                elif event.key == K_s:
                    self.toggle_sonar()

                elif event.key == K_m:
                    self.toggle_mapping()

                elif event.key == K_f:
                    self.mapping_system.toggle_full_map()

                elif event.key == K_d:
                    self.mapping_system.toggle_debug_info()

                elif event.key == K_UP:
                    new_speed = self.mapping_system.increase_robot_speed()
                    if self.robot:
                        self.robot.speed = 3.0 * new_speed
                    print(f"Robot speed: x{new_speed}")

                elif event.key == K_DOWN:
                    new_speed = self.mapping_system.decrease_robot_speed()
                    if self.robot:
                        self.robot.speed = 3.0 * new_speed
                    print(f"Robot speed: x{new_speed}")

                elif event.key == K_RIGHT:
                    new_range = self.mapping_system.increase_sonar_range()
                    if self.robot:
                        self.robot.sensor_range = 120 * new_range
                    print(f"Sonar range: x{new_range}")

                elif event.key == K_LEFT:
                    new_range = self.mapping_system.decrease_sonar_range()
                    if self.robot:
                        self.robot.sensor_range = 120 * new_range
                    print(f"Sonar range: x{new_range}")

                elif event.key == K_PLUS or event.key == K_EQUALS:
                    new_alpha = self.mapping_system.increase_fog_transparency()
                    print(f"Fog transparency: {(255 - new_alpha) / 255.0:.2f}")

                elif event.key == K_MINUS:
                    new_alpha = self.mapping_system.decrease_fog_transparency()
                    print(f"Fog transparency: {(255 - new_alpha) / 255.0:.2f}")

                elif event.key == K_b:
                    self.running = False
                    print("Returning to menu")

            elif event.type == MOUSEBUTTONDOWN:
                if event.button == 1:
                    # No buttons to check in this version

                    # Handle map clicks after buttons
                    self.handle_mouse_click(event.pos)

    def reset_simulation(self):
        """Reset the simulation to initial state."""
        self.start_pos = None
        self.end_pos = None
        self.robot = None
        self.navigator.reset_path()
        self.setting_start = True
        self.mapping_system.reset()
        self.destination_reached = False  # Reset destination flag
        print("Simulation reset")

    def toggle_sonar(self):
        """Toggle sonar visualization on/off."""
        self.show_sonar = not self.show_sonar
        print(f"Sonar visualization: {'ON' if self.show_sonar else 'OFF'}")

    def toggle_mapping(self):
        """Toggle mapping functionality on/off."""
        self.mapping_enabled = not self.mapping_enabled
        print(f"Mapping: {'ON' if self.mapping_enabled else 'OFF'}")

    def handle_mouse_click(self, pos):
        """Handle mouse click for setting start/end positions."""
        if self.setting_start:
            # Set start position
            if self.navigator.is_position_safe(pos):
                self.start_pos = pos
                # Create the robot at the start position with adjustable parameters
                self.robot = Robot(position=pos, size=40)

                # Apply speed and range multipliers
                self.robot.speed = 3.0 * self.mapping_system.robot_speed_multiplier
                self.robot.sensor_range = 120 * self.mapping_system.sonar_range_multiplier

                self.setting_start = False
                self.destination_reached = False  # Reset destination flag when setting a new start
                print(f"Start position set at {pos}")
            else:
                print("Cannot start in an obstacle. Try again.")

        else:
            # Set end position
            if self.navigator.is_position_safe(pos):
                self.end_pos = pos
                self.destination_reached = False  # Reset destination flag when setting a new end

                # Find path
                if self.robot:
                    try:
                        path = self.navigator.find_path(self.robot.position, self.end_pos)
                        if path:
                            print(f"Path found with {len(path)} waypoints")
                        else:
                            print("No path found!")
                    except Exception as e:
                        print(f"Error finding path: {e}")

                # Reset for next route
                self.setting_start = True
                print(f"End position set at {pos}")
            else:
                print("Cannot end in an obstacle. Try again.")
                self.end_pos = None

    def update(self):
        """Update simulation state."""
        if not self.robot:
            return

        # Update robot movement
        self._update_robot_movement()

        # Update mapping with current sonar data
        if self.mapping_enabled and self.robot:
            # Pass width and height separately
            obstacle_points = self.robot.detect_obstacles(
                self.navigator.obstacle_map,
                self.navigator.screen_width,
                self.navigator.screen_height
            )

            # Update the mapping system
            self.mapping_system.update_visibility_from_sonar(
                self.robot.position,
                obstacle_points,
                self.robot.sensor_range
            )

    def _update_robot_movement(self):
        """Handle robot movement and pathfinding."""
        # If destination is already reached, don't continue with navigation
        if self.destination_reached:
            # Only perform exploration if in mapping mode without a target
            if self.mapping_enabled and not self.end_pos:
                self._explore_environment()
            return

        # Check for obstacles that need avoidance
        self.robot.check_for_obstacles(self.navigator.obstacle_map,
                                       (self.navigator.screen_width, self.navigator.screen_height))

        # If robot is in obstacle avoidance mode
        if self.robot.avoiding_obstacle:
            # Perform avoidance maneuver
            recalculate = self.robot.avoid_obstacles(self.navigator.is_position_safe)

            # If avoidance is complete, recalculate path
            if recalculate and self.end_pos and not self.destination_reached:
                path = self.navigator.find_path(self.robot.position, self.end_pos)
                if path:
                    print("Recalculated path after obstacle avoidance")
                else:
                    print("Failed to recalculate path!")

            return

        # Check if robot is stuck
        if self.robot.check_if_stuck(self.navigator.is_position_safe, self.end_pos):
            # If stuck was detected and handled, skip normal movement
            return

        # If we have an end position, follow the path
        if self.end_pos:
            # Normal path following
            current_waypoint = self.navigator.get_current_waypoint()
            if current_waypoint:
                # Move towards current waypoint
                reached = self.robot.move_towards_waypoint(current_waypoint, self.navigator.is_position_safe)

                # If reached waypoint, move to next one
                if reached:
                    if not self.navigator.advance_to_next_waypoint():
                        print("Destination reached!")
                        self.destination_reached = True  # Set flag to indicate destination reached
                        self.robot.consecutive_stucks = 0  # Reset stuck counter once destination is reached
                        self.robot.stuck_time = 0  # Reset stuck timer
            else:
                # No waypoints - already reached or no path
                if self.navigator.path:
                    print("No current waypoint but path exists - likely reached destination")
                    self.destination_reached = True
        else:
            # Exploration mode - robot just wanders around mapping the environment
            if not self.robot.avoiding_obstacle:
                self._explore_environment()

    def _explore_environment(self):
        """Logic for autonomous exploration (when no endpoint is set)."""
        # For now, this is a simple random wandering behavior
        # This could be enhanced with actual exploration algorithms

        # Occasionally change direction randomly
        if np.random.random() < 0.02:  # 2% chance per frame
            self.robot.heading += (np.random.random() - 0.5) * math.pi / 2

        # Move forward
        new_pos = (self.robot.position[0] + self.robot.speed * math.cos(self.robot.heading),
                   self.robot.position[1] + self.robot.speed * math.sin(self.robot.heading))

        # Check if new position is safe
        if self.navigator.is_position_safe(new_pos):
            self.robot.position = new_pos
        else:
            # If we'd hit a wall, turn away
            self.robot.heading += math.pi / 4  # Turn 45 degrees

    def draw(self):
        """Draw everything on the screen."""
        # Draw map
        self.screen.blit(self.map_img, (0, 0))

        # No buttons in this version

        # Draw path
        self.navigator.draw_path(self.screen)

        # Draw start and end positions
        if self.start_pos:
            pygame.draw.circle(self.screen, BLUE, self.start_pos, 10)

        if self.end_pos:
            pygame.draw.circle(self.screen, RED, self.end_pos, 10)

        # Draw robot and sensor data
        if self.robot:
            if self.show_sonar:
                self.robot.draw_sensor_data(self.screen)
            self.robot.draw(self.screen)

        # Apply mapping effect
        if self.mapping_enabled:
            self.mapping_system.draw(self.screen, self.map_img)

        # Draw UI elements
        self.draw_ui()

        # Update display
        pygame.display.flip()

    def draw_ui(self):
        """Draw user interface elements."""
        # Draw instructions - now without background
        if self.setting_start:
            text = self.font.render('Click to set START position', True, BLACK)
        else:
            text = self.font.render('Click to set END position', True, BLACK)

        self.screen.blit(text, (20, 15))

        # Draw controls info
        controls_bg = pygame.Surface((380, 180), pygame.SRCALPHA)
        controls_bg.fill((255, 255, 255, 200))
        self.screen.blit(controls_bg, (self.navigator.screen_width - 390, 10))

        debug_state = "ON" if self.mapping_system.show_debug_info else "OFF"
        fog_value = f"{(255 - self.mapping_system.fog_alpha) / 255.0:.2f}"
        speed_val = f"x{self.mapping_system.robot_speed_multiplier:.1f}"
        sonar_val = f"x{self.mapping_system.sonar_range_multiplier:.1f}"

        controls = [
            f"Sonar: {'ON' if self.show_sonar else 'OFF'} (S)",
            f"Mapping: {'ON' if self.mapping_enabled else 'OFF'} (M)",
            "Full Map: F",
            f"Debug Info (D): {debug_state}",
            f"Speed (UP/DOWN): {speed_val}",
            f"Sonar Range (LEFT/RIGHT): {sonar_val}",
            f"Fog (+/-): {fog_value}",
            "Reset: R",
            "Back to Menu: ESC or B"
        ]

        for i, ctrl in enumerate(controls):
            ctrl_text = self.font.render(ctrl, True, BLACK)
            self.screen.blit(ctrl_text, (self.navigator.screen_width - 380, 15 + i * 20))

        # Draw destination reached message if applicable
        if self.destination_reached:
            status_bg = pygame.Surface((300, 30), pygame.SRCALPHA)
            status_bg.fill((0, 200, 0, 180))
            self.screen.blit(status_bg, (self.navigator.screen_width - 310, 200))

            status_text = self.font.render('Destination reached!', True, WHITE)
            self.screen.blit(status_text, (self.navigator.screen_width - 300, 205))
        # Draw obstacle avoidance status if active
        elif self.robot and self.robot.avoiding_obstacle:
            status_bg = pygame.Surface((300, 30), pygame.SRCALPHA)
            status_bg.fill((255, 0, 0, 180))
            self.screen.blit(status_bg, (self.navigator.screen_width - 310, 200))

            status_text = self.font.render(f'Avoiding obstacle! ({self.robot.avoid_timer})', True, WHITE)
            self.screen.blit(status_text, (self.navigator.screen_width - 300, 205))

        # Draw stuck counter if robot is getting repeatedly stuck
        if self.robot and self.robot.consecutive_stucks > 0:
            stuck_bg = pygame.Surface((320, 30), pygame.SRCALPHA)
            stuck_bg.fill((200, 0, 200, 180))
            self.screen.blit(stuck_bg, (self.navigator.screen_width - 330, 240))

            stuck_text = self.font.render(
                f'Stuck recovery attempts: {self.robot.consecutive_stucks}/{self.robot.max_consecutive_stucks}',
                True, WHITE)
            self.screen.blit(stuck_text, (self.navigator.screen_width - 320, 245))

    def run(self):
        """Main application loop."""
        while self.running:
            # Process events
            self.handle_events()

            # Update simulation
            self.update()

            # Draw everything
            self.draw()

            # Cap at 60 FPS
            self.clock.tick(60)


# Run the application
if __name__ == "__main__":
    app = RobotNavigationMapping()
    app.run()

    # Clean up pygame
    pygame.quit()
    print("Application closed")