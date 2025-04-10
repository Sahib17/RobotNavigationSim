import pygame
import sys
import os
from pygame.locals import *
import robot_navigation_main

# Initialize pygame
pygame.init()

# Screen dimensions
SCREEN_WIDTH = 1000  # Increased width to accommodate all elements
SCREEN_HEIGHT = 700  # Increased height to accommodate all elements

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
GRAY = (200, 200, 200)
DARK_GRAY = (100, 100, 100)
BLUE = (0, 0, 255)
GREEN = (0, 200, 0)
LIGHT_BLUE = (100, 100, 255)
LIGHT_GREEN = (100, 255, 100)
PURPLE = (128, 0, 128)

# Font
FONT = pygame.font.SysFont(None, 32)
SMALL_FONT = pygame.font.SysFont(None, 24)


class Button:
    """Button class for menu interaction."""

    def __init__(self, x, y, width, height, text, color=BLUE, hover_color=LIGHT_BLUE, text_color=WHITE):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.color = color
        self.hover_color = hover_color
        self.text_color = text_color
        self.hovered = False

    def draw(self, screen):
        # Draw button rectangle
        color = self.hover_color if self.hovered else self.color
        pygame.draw.rect(screen, color, self.rect)
        pygame.draw.rect(screen, BLACK, self.rect, 2)  # Border

        # Draw text
        text_surf = FONT.render(self.text, True, self.text_color)
        text_rect = text_surf.get_rect(center=self.rect.center)
        screen.blit(text_surf, text_rect)

    def check_hover(self, mouse_pos):
        self.hovered = self.rect.collidepoint(mouse_pos)
        return self.hovered

    def check_click(self, mouse_pos, mouse_click):
        return self.rect.collidepoint(mouse_pos) and mouse_click


class RadioButton:
    """Radio button for exclusive selection."""

    def __init__(self, x, y, radius, text, group, selected=False):
        self.x = x
        self.y = y
        self.radius = radius
        self.text = text
        self.group = group
        self.selected = selected

    def draw(self, screen):
        # Draw outer circle
        pygame.draw.circle(screen, BLACK, (self.x, self.y), self.radius, 2)

        # Draw inner circle if selected
        if self.selected:
            pygame.draw.circle(screen, BLUE, (self.x, self.y), self.radius - 4)

        # Draw text
        text_surf = FONT.render(self.text, True, BLACK)
        text_rect = text_surf.get_rect(midleft=(self.x + self.radius + 10, self.y))
        screen.blit(text_surf, text_rect)

    def check_click(self, mouse_pos, mouse_click):
        # Calculate distance from center to mouse
        distance = ((mouse_pos[0] - self.x) ** 2 + (mouse_pos[1] - self.y) ** 2) ** 0.5

        # Return true if clicked within radius
        return distance <= self.radius and mouse_click

    def select(self):
        # Deselect all other buttons in the group
        for button in self.group:
            button.selected = False

        # Select this button
        self.selected = True


class Dropdown:
    """Dropdown menu for map selection."""

    def __init__(self, x, y, width, height, options):
        self.rect = pygame.Rect(x, y, width, height)
        self.options = options
        self.open = False
        self.selected_index = 0
        self.option_height = height
        self.hovered_index = -1

        # Create a separate surface for the dropdown options
        self.max_visible_options = 5  # Maximum number of visible options at once
        self.dropdown_surface = None
        self.update_dropdown_surface()

    def update_dropdown_surface(self):
        """Create or update the surface for dropdown options."""
        visible_options = min(len(self.options), self.max_visible_options)
        surface_height = visible_options * self.option_height

        if surface_height > 0:
            self.dropdown_surface = pygame.Surface((self.rect.width, surface_height))
            self.dropdown_surface.fill(WHITE)

            for i in range(visible_options):
                option_rect = pygame.Rect(
                    0,
                    i * self.option_height,
                    self.rect.width,
                    self.option_height
                )

                # Highlight hovered option
                if i == self.hovered_index:
                    pygame.draw.rect(self.dropdown_surface, LIGHT_BLUE, option_rect)
                else:
                    pygame.draw.rect(self.dropdown_surface, WHITE, option_rect)

                pygame.draw.rect(self.dropdown_surface, BLACK, option_rect, 1)

                # Draw option text
                text_surf = FONT.render(self.options[i], True, BLACK)
                text_rect = text_surf.get_rect(midleft=(10, option_rect.centery))
                self.dropdown_surface.blit(text_surf, text_rect)

    def get_selected(self):
        if 0 <= self.selected_index < len(self.options):
            return self.options[self.selected_index]
        return None

    def draw(self, screen):
        # Draw the main dropdown box
        pygame.draw.rect(screen, WHITE, self.rect)
        pygame.draw.rect(screen, BLACK, self.rect, 2)

        # Draw the selected option
        if self.options:
            selected_text = self.options[self.selected_index]
            text_surf = FONT.render(selected_text, True, BLACK)
            text_rect = text_surf.get_rect(midleft=(self.rect.x + 10, self.rect.centery))
            screen.blit(text_surf, text_rect)

        # Draw dropdown arrow
        arrow_points = [
            (self.rect.right - 20, self.rect.centery - 5),
            (self.rect.right - 10, self.rect.centery + 5),
            (self.rect.right - 30, self.rect.centery + 5)
        ]
        pygame.draw.polygon(screen, BLACK, arrow_points)

        # Draw dropdown options if open
        if self.open and self.dropdown_surface:
            # Position the dropdown below the main box
            dropdown_rect = pygame.Rect(
                self.rect.x,
                self.rect.y + self.rect.height,
                self.rect.width,
                self.dropdown_surface.get_height()
            )

            # Ensure dropdown doesn't go below the screen
            if dropdown_rect.bottom > SCREEN_HEIGHT:
                dropdown_rect.y = self.rect.y - dropdown_rect.height

            screen.blit(self.dropdown_surface, dropdown_rect.topleft)

    def check_hover(self, mouse_pos):
        # Check if main dropdown is hovered
        if self.rect.collidepoint(mouse_pos):
            return True

        # Check if any dropdown option is hovered
        if self.open and self.dropdown_surface:
            dropdown_rect = pygame.Rect(
                self.rect.x,
                self.rect.y + self.rect.height,
                self.rect.width,
                self.dropdown_surface.get_height()
            )

            # Ensure dropdown doesn't go below the screen
            if dropdown_rect.bottom > SCREEN_HEIGHT:
                dropdown_rect.y = self.rect.y - dropdown_rect.height

            if dropdown_rect.collidepoint(mouse_pos):
                # Calculate which option is being hovered
                relative_y = mouse_pos[1] - dropdown_rect.y
                self.hovered_index = min(relative_y // self.option_height,
                                         len(self.options) - 1)
                self.update_dropdown_surface()
                return True

        self.hovered_index = -1
        return False

    def handle_event(self, event, mouse_pos):
        if event.type == MOUSEBUTTONDOWN and event.button == 1:
            # Toggle dropdown if clicked
            if self.rect.collidepoint(mouse_pos):
                self.open = not self.open
                if self.open:
                    self.update_dropdown_surface()
                return True

            # Check if an option was clicked
            if self.open and self.dropdown_surface:
                dropdown_rect = pygame.Rect(
                    self.rect.x,
                    self.rect.y + self.rect.height,
                    self.rect.width,
                    self.dropdown_surface.get_height()
                )

                # Ensure dropdown doesn't go below the screen
                if dropdown_rect.bottom > SCREEN_HEIGHT:
                    dropdown_rect.y = self.rect.y - dropdown_rect.height

                if dropdown_rect.collidepoint(mouse_pos):
                    # Calculate which option was clicked
                    relative_y = mouse_pos[1] - dropdown_rect.y
                    clicked_index = relative_y // self.option_height

                    # Make sure the index is valid
                    if 0 <= clicked_index < len(self.options):
                        self.selected_index = clicked_index
                        self.open = False
                        return True

            # Close dropdown if clicked elsewhere
            self.open = False

        return False


class MapPreview:
    """Preview display for the selected map."""

    def __init__(self, x, y, width, height):
        self.rect = pygame.Rect(x, y, width, height)
        self.image = None
        self.map_name = None

    def set_map(self, map_name):
        if self.map_name != map_name:
            self.map_name = map_name
            try:
                # Load the map image
                full_path = os.path.join("maps", map_name)
                original_image = pygame.image.load(full_path)

                # Scale image to fit the preview box while maintaining aspect ratio
                img_width, img_height = original_image.get_size()
                scale = min(self.rect.width / img_width, self.rect.height / img_height)
                new_width = int(img_width * scale)
                new_height = int(img_height * scale)

                self.image = pygame.transform.scale(original_image, (new_width, new_height))
            except pygame.error:
                print(f"Error loading map: {map_name}")
                self.image = None

    def draw(self, screen):
        # Draw background
        pygame.draw.rect(screen, GRAY, self.rect)
        pygame.draw.rect(screen, BLACK, self.rect, 2)

        # Draw image if available
        if self.image:
            # Center the image in the preview box
            image_rect = self.image.get_rect()
            image_rect.center = self.rect.center
            screen.blit(self.image, image_rect)
        else:
            # Draw "No Preview" text
            text_surf = FONT.render("No Preview Available", True, BLACK)
            text_rect = text_surf.get_rect(center=self.rect.center)
            screen.blit(text_surf, text_rect)


def get_map_files():
    """Get list of map files from the maps directory."""
    try:
        map_files = []
        if os.path.exists("maps"):
            for filename in os.listdir("maps"):
                if filename.lower().endswith(('.png', '.jpg', '.jpeg')):
                    map_files.append(filename)
        return map_files
    except Exception as e:
        print(f"Error reading map files: {e}")
        return []


def start_simulation(map_filename, algorithm="astar", use_mapping_mode=False):
    """Start the robot navigation simulation with the selected map, algorithm, and mode."""
    # Ensure the maps directory exists
    if not os.path.exists("maps"):
        os.makedirs("maps")

    try:
        map_path = os.path.join("maps", map_filename)

        # Set the map file for the simulation to use
        robot_navigation_main.MAP_FILENAME = map_path

        # Create the simulation instance
        sim = robot_navigation_main.RobotNavigationMapping()

        # Update mapping mode based on selection
        sim.mapping_enabled = use_mapping_mode

        # Run the simulation
        sim.run()
    except Exception as e:
        print(f"Error starting simulation: {e}")
        import traceback
        traceback.print_exc()


def main_menu():
    """Main menu screen with map selection and algorithm choice."""
    # Set up display
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("Robot Navigation - Main Menu")

    # Get maps
    map_files = get_map_files()
    if not map_files:
        map_files = ["default.png"]  # Fallback option

    # Create UI elements
    title_font = pygame.font.SysFont(None, 48)
    title_text = title_font.render("Robot Navigation Simulator", True, BLACK)
    title_rect = title_text.get_rect(center=(SCREEN_WIDTH // 2, 60))

    map_label = FONT.render("Select Map:", True, BLACK)
    map_label_rect = map_label.get_rect(topleft=(50, 140))

    dropdown = Dropdown(50, 180, 350, 40, map_files)

    # Place map preview on the right side
    map_preview = MapPreview(550, 140, 400, 300)

    # Create algorithm selection radio buttons
    algo_label = FONT.render("Navigation Algorithm:", True, BLACK)
    algo_label_rect = algo_label.get_rect(topleft=(50, 260))

    radio_buttons = []

    # A* algorithm (default)
    astar_button = RadioButton(70, 310, 12, "A* Algorithm", None, True)
    # Dijkstra algorithm
    dijkstra_button = RadioButton(70, 350, 12, "Dijkstra's Algorithm", None, False)
    # Grassfire algorithm
    grassfire_button = RadioButton(70, 390, 12, "Grassfire Algorithm", None, False)
    # Add new advanced algorithms
    density_button = RadioButton(70, 430, 12, "Obstacle Density Path", None, False)
    sonar_button = RadioButton(70, 470, 12, "Sonar Guided Path", None, False)

    # Group the radio buttons
    radio_group = [astar_button, dijkstra_button, grassfire_button, density_button, sonar_button]
    for rb in radio_group:
        rb.group = radio_group
        radio_buttons.append(rb)

    # Create mode radio buttons
    mode_label = FONT.render("Simulator Mode:", True, BLACK)
    mode_label_rect = mode_label.get_rect(topleft=(50, 520))

    # Mode selection
    navigation_button = RadioButton(70, 560, 12, "Regular Navigation", None, True)
    mapping_button = RadioButton(350, 560, 12, "Mapping Mode", None, False)

    # Group the mode buttons
    mode_group = [navigation_button, mapping_button]
    for rb in mode_group:
        rb.group = mode_group
        radio_buttons.append(rb)

    # Info texts
    info_text1 = SMALL_FONT.render("A*: Balance between speed and optimal path", True, DARK_GRAY)
    info_text2 = SMALL_FONT.render("Dijkstra: Finds shortest path, slower than A*", True, DARK_GRAY)
    info_text3 = SMALL_FONT.render("Grassfire: Simple flood-fill method for maze solving", True, DARK_GRAY)
    info_text4 = SMALL_FONT.render("Obstacle Density: Finds paths away from obstacles", True, DARK_GRAY)
    info_text5 = SMALL_FONT.render("Sonar Guided: Uses 360° vision for smart navigation", True, DARK_GRAY)

    info_rect1 = info_text1.get_rect(topleft=(100, 310 + 15))
    info_rect2 = info_text2.get_rect(topleft=(100, 350 + 15))
    info_rect3 = info_text3.get_rect(topleft=(100, 390 + 15))
    info_rect4 = info_text4.get_rect(topleft=(100, 430 + 15))
    info_rect5 = info_text5.get_rect(topleft=(100, 470 + 15))

    # Mode info texts
    mode_info1 = SMALL_FONT.render("Standard robot navigation", True, DARK_GRAY)
    mode_info2 = SMALL_FONT.render("Robot maps out its environment", True, DARK_GRAY)

    mode_info_rect1 = mode_info1.get_rect(topleft=(90, 560 + 15))
    mode_info_rect2 = mode_info2.get_rect(topleft=(370, 560 + 15))

    # Create buttons
    start_button = Button(SCREEN_WIDTH // 2 - 100, 620, 200, 60, "Start Simulation", GREEN, LIGHT_GREEN)
    exit_button = Button(SCREEN_WIDTH - 120, 30, 100, 40, "Exit", DARK_GRAY)

    # Set initial map preview
    if map_files:
        map_preview.set_map(map_files[0])

    clock = pygame.time.Clock()

    # Main menu loop
    running = True
    while running:
        mouse_pos = pygame.mouse.get_pos()

        # Event handling
        for event in pygame.event.get():
            if event.type == QUIT:
                pygame.quit()
                sys.exit()

            # Handle dropdown interaction
            if dropdown.handle_event(event, mouse_pos):
                selected_map = dropdown.get_selected()
                if selected_map:
                    map_preview.set_map(selected_map)

            # Handle radio button clicks
            if event.type == MOUSEBUTTONDOWN and event.button == 1:
                for button in radio_buttons:
                    if button.check_click(mouse_pos, True):
                        button.select()

                # Handle regular button clicks
                if exit_button.check_click(mouse_pos, True):
                    pygame.quit()
                    sys.exit()

                if start_button.check_click(mouse_pos, True):
                    selected_map = dropdown.get_selected()

                    # Determine selected algorithm
                    selected_algorithm = "astar"  # Default
                    if dijkstra_button.selected:
                        selected_algorithm = "dijkstra"
                    elif grassfire_button.selected:
                        selected_algorithm = "grassfire"
                    elif density_button.selected:
                        selected_algorithm = "density"
                    elif sonar_button.selected:
                        selected_algorithm = "sonar"

                    # Determine selected mode
                    use_mapping_mode = mapping_button.selected

                    if selected_map:
                        # Start the simulation with the selected map, algorithm, and mode
                        print(
                            f"Starting simulation with map: {selected_map}, algorithm: {selected_algorithm}, mapping mode: {use_mapping_mode}")
                        start_simulation(selected_map, selected_algorithm, use_mapping_mode)

                        # Reset display after returning from simulation
                        screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
                        pygame.display.set_caption("Robot Navigation - Main Menu")

        # Update UI hover states
        start_button.check_hover(mouse_pos)
        exit_button.check_hover(mouse_pos)
        dropdown.check_hover(mouse_pos)

        # Draw everything
        screen.fill(WHITE)

        # Title
        screen.blit(title_text, title_rect)

        # Map selection
        screen.blit(map_label, map_label_rect)

        # Algorithm selection
        screen.blit(algo_label, algo_label_rect)
        for button in radio_buttons[:5]:  # First 5 buttons are algorithm selection
            button.draw(screen)

        # Mode selection
        screen.blit(mode_label, mode_label_rect)
        for button in radio_buttons[5:]:  # Last 2 buttons are mode selection
            button.draw(screen)

        # Info texts
        screen.blit(info_text1, info_rect1)
        screen.blit(info_text2, info_rect2)
        screen.blit(info_text3, info_rect3)
        screen.blit(info_text4, info_rect4)
        screen.blit(info_text5, info_rect5)

        # Mode info texts
        screen.blit(mode_info1, mode_info_rect1)
        screen.blit(mode_info2, mode_info_rect2)

        # Map preview (on the right)
        map_preview.draw(screen)

        # Buttons
        start_button.draw(screen)
        exit_button.draw(screen)

        # Draw dropdown last to make sure it appears on top
        dropdown.draw(screen)

        pygame.display.flip()
        clock.tick(60)


if __name__ == "__main__":
    # Create maps directory if it doesn't exist
    if not os.path.exists("maps"):
        os.makedirs("maps")
        print("Created 'maps' directory. Please add map images to this folder.")

    # Start the menu
    main_menu()