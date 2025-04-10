import pygame
import math
import numpy as np


class Robot:
    """Robot class that manages robot movement, sensing and obstacle avoidance."""

    def __init__(self, position, size=40, heading=0):
        self.position = position
        self.size = size
        self.heading = heading  # radians
        self.speed = 3.0  # base speed in pixels per frame
        self.turn_speed = 0.1  # radians per frame

        # Obstacle avoidance properties
        self.avoiding_obstacle = False
        self.avoid_timer = 0
        self.avoid_direction = 1  # 1=right, -1=left
        self.min_obstacle_dist = 40  # Distance at which to start avoiding obstacles
        self.avoid_maneuver_time = 30  # Duration of avoidance maneuver

        # Status tracking
        self.last_position = None
        self.stuck_time = 0
        self.consecutive_stucks = 0
        self.max_consecutive_stucks = 3
        self.last_detected_obstacles = []

        # Sensor configuration
        self.sensor_range = 120  # How far the sonar can detect
        self.ray_count = 16  # Number of rays to cast (more for better detection)
        self.use_360_sonar = True  # Enable 360-degree detection

        # Try to load robot image
        self.image = None
        try:
            self.image = pygame.image.load('robot.png')
            self.image = pygame.transform.scale(self.image, (size, size))
        except pygame.error:
            print("Robot image not found. Using placeholder.")

    def detect_obstacles(self, obstacle_map, screen_width, screen_height):
        """Detect obstacles using ray casting with 360-degree coverage."""
        if obstacle_map is None or self.position is None:
            return []

        detection_points = []
        x, y = self.position

        # Cast rays in all directions when using 360-degree sonar
        if self.use_360_sonar:
            angle_span = 2 * math.pi  # Full 360 degrees
            start_angle = self.heading - math.pi  # Start casting from behind
        else:
            # Default to front-facing cone if 360 is disabled
            angle_span = math.pi / 2  # 90-degree cone
            start_angle = self.heading - angle_span / 2  # Center on heading

        angle_step = angle_span / self.ray_count

        for i in range(self.ray_count):
            angle = start_angle + i * angle_step

            # FIX: Convert sensor_range to integer for range() function
            max_range = int(self.sensor_range)

            # Use smaller step size for more reliable detection
            for dist in range(5, max_range, 5):
                ray_x = int(x + dist * math.cos(angle))
                ray_y = int(y + dist * math.sin(angle))

                # Check if point is out of bounds
                if (ray_x < 0 or ray_x >= screen_width or
                        ray_y < 0 or ray_y >= screen_height):
                    break

                # Check if hit obstacle
                if obstacle_map[ray_y, ray_x]:
                    detection_points.append((ray_x, ray_y, dist, angle))
                    break

                # If at max range, add an indicator point
                if dist >= max_range - 5:
                    detection_points.append((ray_x, ray_y, -dist, angle))
                    break

        self.last_detected_obstacles = detection_points
        return detection_points

    def find_open_directions(self, obstacle_points):
        """Analyze obstacle data to find open directions."""
        # Create a map of obstacle distances by angle
        angle_map = {}

        # Process all detected points
        for point in obstacle_points:
            if len(point) < 4 or point[2] <= 0:
                continue  # Skip points with invalid data

            x, y, dist, angle = point

            # Normalize angle to [0, 2π)
            while angle < 0:
                angle += 2 * math.pi
            while angle >= 2 * math.pi:
                angle -= 2 * math.pi

            # Store the distance to obstacle at this angle
            if angle not in angle_map or dist < angle_map[angle]:
                angle_map[angle] = dist

        # Identify "slices" of open directions
        open_slices = []
        curr_slice_start = None
        prev_angle = None

        # Divide the circle into 36 segments (every 10 degrees)
        segments = 36
        for i in range(segments + 1):  # +1 to complete the circle
            test_angle = i * (2 * math.pi / segments)
            # Find closest measured angle
            closest_measured = min(angle_map.keys(), key=lambda a: abs(a - test_angle)) if angle_map else None

            is_open = (closest_measured is None or
                       abs(closest_measured - test_angle) > 0.2 or  # Too far from measurement
                       angle_map[closest_measured] > self.min_obstacle_dist * 1.5)  # Enough space

            if is_open and curr_slice_start is None:
                # Start of a new open slice
                curr_slice_start = test_angle
            elif not is_open and curr_slice_start is not None:
                # End of current open slice
                open_slices.append((curr_slice_start, prev_angle))
                curr_slice_start = None

            prev_angle = test_angle

        # Handle case where an open slice wraps around
        if curr_slice_start is not None:
            open_slices.append((curr_slice_start, prev_angle))

        return open_slices

    def check_for_obstacles(self, obstacle_map, screen_dimensions):
        """Check if there are obstacles that require evasive action."""
        width, height = screen_dimensions

        # Only run obstacle check if not already avoiding
        if not self.avoiding_obstacle:
            # Get obstacle detections with 360-degree coverage
            obstacle_points = self.detect_obstacles(obstacle_map, width, height)

            # Create front detection cone (narrower than the full 360)
            front_obstacles = []

            for point in obstacle_points:
                if len(point) < 4 or point[2] <= 0:  # Skip maximum range points
                    continue

                x, y, dist, angle = point

                # Calculate angle difference relative to robot heading
                angle_diff = angle - self.heading
                # Normalize to [-π, π]
                while angle_diff > math.pi: angle_diff -= 2 * math.pi
                while angle_diff < -math.pi: angle_diff += 2 * math.pi

                # Only consider obstacles directly in front (30 degree cone)
                if abs(angle_diff) < math.pi / 6 and dist < self.min_obstacle_dist:
                    front_obstacles.append((x, y, dist, angle_diff))

            # If there are obstacles in front, start avoidance
            if len(front_obstacles) > 0:
                print(f"Obstacle detected! {len(front_obstacles)} obstacles in front")
                self.avoiding_obstacle = True
                self.avoid_timer = self.avoid_maneuver_time

                # Find open directions using 360-degree data
                open_directions = self.find_open_directions(obstacle_points)

                if open_directions:
                    # Choose the best direction to turn
                    best_direction = self.choose_best_avoidance_direction(open_directions)

                    # Calculate if we should turn left or right
                    angle_diff = best_direction - self.heading
                    # Normalize to [-π, π]
                    while angle_diff > math.pi: angle_diff -= 2 * math.pi
                    while angle_diff < -math.pi: angle_diff += 2 * math.pi

                    self.avoid_direction = 1 if angle_diff > 0 else -1
                    print(f"Turning {self.avoid_direction} to open direction")
                else:
                    # No good open directions, use simple avoidance
                    # Turn away from first detected obstacle
                    if front_obstacles:
                        _, _, _, angle_diff = front_obstacles[0]
                        self.avoid_direction = 1 if angle_diff > 0 else -1
                        print(f"No clear path, turning {self.avoid_direction}")

    def choose_best_avoidance_direction(self, open_slices):
        """Choose the best direction to move based on open slices."""
        if not open_slices:
            # Default to current heading if no open slices
            return self.heading

        # Try to find a slice that contains a direction close to our desired heading
        desired_heading = self.heading  # Default: maintain current heading

        # For each slice, find the angle in the slice closest to our desired heading
        best_angle = None
        best_diff = float('inf')

        for start_angle, end_angle in open_slices:
            # Check if the slice wraps around
            if end_angle < start_angle:
                end_angle += 2 * math.pi

            # Check if desired heading is in this slice
            normalized_heading = desired_heading
            while normalized_heading < start_angle:
                normalized_heading += 2 * math.pi
            while normalized_heading >= 2 * math.pi:
                normalized_heading -= 2 * math.pi

            if start_angle <= normalized_heading <= end_angle:
                # Desired heading is in this slice
                return desired_heading

            # Otherwise, find closest angle in the slice
            closest_angle = start_angle if abs(start_angle - normalized_heading) < abs(
                end_angle - normalized_heading) else end_angle
            diff = abs(closest_angle - normalized_heading)

            if diff < best_diff:
                best_diff = diff
                best_angle = closest_angle

        return best_angle if best_angle is not None else desired_heading

    def avoid_obstacles(self, is_position_safe_func):
        """Perform obstacle avoidance maneuver."""
        # Decrease avoidance timer
        self.avoid_timer -= 1

        if self.avoid_timer <= 0:
            # End avoidance maneuver
            self.avoiding_obstacle = False
            print("Ending avoidance maneuver")

            return True  # Signal to recalculate path
        else:
            # Improved avoidance: back up and turn
            # First half: back up slightly
            if self.avoid_timer > self.avoid_maneuver_time / 2:
                # Back up
                backup_speed = 1.5
                new_pos = (self.position[0] - backup_speed * math.cos(self.heading),
                           self.position[1] - backup_speed * math.sin(self.heading))

                if is_position_safe_func(new_pos):
                    self.position = new_pos

            # Second half: turn away from obstacle
            else:
                # Turn in the chosen direction
                turn_amount = 0.12
                self.heading += self.avoid_direction * turn_amount

        return False  # Don't recalculate path yet

    def check_if_stuck(self, is_position_safe_func, end_pos=None, find_path_func=None):
        """Check if robot is stuck and handle it if necessary."""
        if self.last_position:
            # Calculate movement since last frame
            move_dist = math.sqrt((self.position[0] - self.last_position[0]) ** 2 +
                                  (self.position[1] - self.last_position[1]) ** 2)

            # If barely moving, increment stuck counter
            if move_dist < 0.5:
                self.stuck_time += 1
            else:
                self.stuck_time = 0

            # If stuck for too long, try to escape
            if self.stuck_time > 60:  # Stuck for 1 second (at 60 FPS)
                print(f"Robot appears stuck, trying to resolve... (Attempt {self.consecutive_stucks + 1})")
                self.consecutive_stucks += 1

                # Start obstacle avoidance mode
                self.avoiding_obstacle = True
                self.avoid_timer = self.avoid_maneuver_time

                # Use 360-degree sensor data to find escape route
                if self.last_detected_obstacles:
                    open_directions = self.find_open_directions(self.last_detected_obstacles)
                    if open_directions:
                        best_direction = self.choose_best_avoidance_direction(open_directions)
                        angle_diff = best_direction - self.heading
                        # Normalize to [-π, π]
                        while angle_diff > math.pi: angle_diff -= 2 * math.pi
                        while angle_diff < -math.pi: angle_diff += 2 * math.pi

                        self.avoid_direction = 1 if angle_diff > 0 else -1
                        print(f"Unstuck: Turning toward open direction: {self.avoid_direction}")
                    else:
                        # No good directions found, alternate
                        self.avoid_direction = 1 if self.consecutive_stucks % 2 == 0 else -1
                else:
                    # No sensor data, alternate direction on consecutive stucks
                    self.avoid_direction = 1 if self.consecutive_stucks % 2 == 0 else -1

                # If stuck too many times, do a more drastic maneuver
                if self.consecutive_stucks > self.max_consecutive_stucks:
                    self._emergency_teleport(is_position_safe_func, end_pos, find_path_func)

                self.stuck_time = 0
                return True  # Signal that the robot was stuck

        # Update last position
        self.last_position = self.position
        return False  # Not stuck

    def _emergency_teleport(self, is_position_safe_func, end_pos, find_path_func=None):
        """Emergency teleport to get out of stuck state."""
        # Reset counters
        self.consecutive_stucks = 0
        self.stuck_time = 0
        self.avoiding_obstacle = False

        # Don't actually teleport, just turn around completely
        self.heading += math.pi  # 180 degree turn
        print("Emergency turnaround - heading in opposite direction")

        return True

    def move_towards_waypoint(self, target, is_position_safe_func, waypoint_tolerance=15):
        """Move robot towards the current waypoint."""
        # Calculate distance and direction to target
        dx = target[0] - self.position[0]
        dy = target[1] - self.position[1]
        distance = math.sqrt(dx * dx + dy * dy)

        # If reached waypoint, signal to move to next one
        if distance < waypoint_tolerance:
            return True

        # Calculate direction to target
        target_angle = math.atan2(dy, dx)

        # Update robot heading
        angle_diff = target_angle - self.heading
        # Normalize angle difference to [-pi, pi]
        while angle_diff > math.pi: angle_diff -= 2 * math.pi
        while angle_diff < -math.pi: angle_diff += 2 * math.pi

        # Turn robot
        turn_speed = min(self.turn_speed, abs(angle_diff))
        if angle_diff > 0:
            self.heading += turn_speed
        else:
            self.heading -= turn_speed

        # Adjust speed based on angle to target - slow down for turns
        speed_factor = max(0.3, 1 - abs(angle_diff) / math.pi)
        current_speed = self.speed * speed_factor

        # Move robot forward
        new_pos = (self.position[0] + current_speed * math.cos(self.heading),
                   self.position[1] + current_speed * math.sin(self.heading))

        # Check if new position is safe
        if is_position_safe_func(new_pos):
            self.position = new_pos
            return False  # Not reached waypoint yet
        else:
            # If collision detected, trigger obstacle avoidance
            print("Collision detected, starting avoidance...")
            self.avoiding_obstacle = True
            self.avoid_timer = self.avoid_maneuver_time
            self.avoid_direction = 1 if np.random.random() > 0.5 else -1
            return False  # Not reached waypoint yet

    def draw(self, screen, color=(0, 0, 255)):
        """Draw the robot on the screen."""
        if self.image:
            # Rotate robot image
            rotated = pygame.transform.rotozoom(self.image, math.degrees(-self.heading), 1)
            rect = rotated.get_rect(center=self.position)
            screen.blit(rotated, rect)
        else:
            # Draw a triangle representing the robot direction
            robot_size = 20
            points = [
                (self.position[0] + robot_size * math.cos(self.heading),
                 self.position[1] + robot_size * math.sin(self.heading)),
                (self.position[0] + robot_size / 2 * math.cos(self.heading + 2.5),
                 self.position[1] + robot_size / 2 * math.sin(self.heading + 2.5)),
                (self.position[0] + robot_size / 2 * math.cos(self.heading - 2.5),
                 self.position[1] + robot_size / 2 * math.sin(self.heading - 2.5))
            ]
            pygame.draw.polygon(screen, color, points)

    def draw_sensor_data(self, screen, color_detection=(255, 0, 0), color_ray=(255, 255, 0)):
        """Draw the robot's sensor data (sonar rays and detected obstacles)."""
        if not self.position or not self.last_detected_obstacles:
            return

        for point in self.last_detected_obstacles:
            if len(point) >= 3:  # If the point includes distance
                x, y, dist = point[0], point[1], point[2]
                # Draw ray from robot to obstacle
                if dist > 0:  # Normal detection
                    pygame.draw.line(screen, color_ray, self.position, (x, y), 1)
                    # Draw detection point
                    pygame.draw.circle(screen, color_detection, (x, y), 3)
                else:  # Maximum range point (no obstacle detected)
                    # Draw as a lighter line to indicate maximum range
                    pygame.draw.line(screen, (100, 100, 0), self.position, (x, y), 1)

        # Visualize open directions if in avoidance mode
        if self.avoiding_obstacle and self.last_detected_obstacles:
            open_directions = self.find_open_directions(self.last_detected_obstacles)

            # Draw open direction slices
            for start_angle, end_angle in open_directions:
                start_point = (
                    self.position[0] + 30 * math.cos(start_angle),
                    self.position[1] + 30 * math.sin(start_angle)
                )
                end_point = (
                    self.position[0] + 30 * math.cos(end_angle),
                    self.position[1] + 30 * math.sin(end_angle)
                )

                # Draw arc to show open directions
                pygame.draw.arc(
                    screen,
                    (0, 255, 0, 150),  # Green, semi-transparent
                    pygame.Rect(
                        self.position[0] - 30,
                        self.position[1] - 30,
                        60, 60
                    ),
                    start_angle, end_angle,
                    2
                )