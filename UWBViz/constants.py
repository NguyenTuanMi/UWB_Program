# constants.py
import pygame
import os

# Display settings
SCREEN_WIDTH, SCREEN_HEIGHT = 1000, 800
POINT_RADIUS = 7
FONT_SIZE = 30
INITIAL_SCALE = 40
MAX_HISTORY = 50
TRACE_FADE_TIME = 10
RECT_WIDTH, RECT_HEIGHT = 4.44, 4.44      # IMPT: Scale in metres

# Colors
BACKGROUND_COLOR = (255, 255, 255)  # White
GRID_COLOR = (200, 200, 200)       # Light gray
LABEL_COLOR = (100, 100, 100)      # Dark gray
BLUE = (0, 0, 255)
RED = (255, 0, 0)

# Resources
RESOURCES_DIR = "UWB_Manipulation/resources"
BGPIC = os.path.join(RESOURCES_DIR, "field2025_old_overlay.png")
WAYPOINTS_JSON_DEFAULT = "waypoint20x20.json"

MARKEDPOINTS_JSON_DEFAULT = "uwb_trace"
MARKEDWAYPOINTS_JSON_DEFAULT = "waypoints_uwb"

# Tag colors for different UWB tags
TAG_COLORS = [
    (0, 51, 102),   # Navy Blue
    (153, 0, 0),    # Dark Red
    (0, 102, 51),   # Forest Green
    (102, 0, 102),  # Purple
    (153, 76, 0),   # Dark Orange
    (51, 51, 0),    # Olive
    (102, 51, 0),   # Brown
    (51, 0, 51),    # Dark Purple
    (0, 76, 153),   # Royal Blue
    (153, 0, 76),   # Dark Pink
]

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)