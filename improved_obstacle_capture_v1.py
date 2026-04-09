import pygame
import random
import time
from collections import deque
from UWB_Manipulation.UWB_Reader import get_target_position

# =========================
# Configuration
# =========================
WIDTH, HEIGHT = 700, 760
FPS = 60
testing = False
UI_HEIGHT = 120

x_min, x_max = -100, 1000
y_min, y_max = -100, 1000
DIFF = x_max//20
UPDATE_INTERVAL = 0.2
TRACE = 5

# =========================
# Utilities
# =========================
def clamp(v, lo, hi):
    return max(lo, min(v, hi))

def map_to_screen(x, y, offset=(0,0), scale=1):
    x, y = x-offset[0], y-offset[1]
    x, y = x/scale, y/scale
    sx = int((x - x_min) / (x_max - x_min) * WIDTH)
    sy = int(
        UI_HEIGHT
        + (HEIGHT - UI_HEIGHT)
        - (y - y_min) / (y_max - y_min) * (HEIGHT - UI_HEIGHT)
    )
    return sx, sy

def draw_axes(surface, font, tick_range=10):
    # Draw Y-axis (x = 0)
    x0, _ = map_to_screen(0, y_min)
    _, y1 = map_to_screen(0, y_max)
    pygame.draw.line(surface, (0, 0, 0), (x0, y1), (x0, map_to_screen(0, y_min)[1]), 2)

    # Y ticks
    for y in range(int(y_min), int(y_max) + 1, tick_range):
        sx, sy = map_to_screen(0, y)
        pygame.draw.line(surface, (0, 0, 0), (sx - 5, sy), (sx + 5, sy), 1)
        label = font.render(str(y), True, (0, 0, 0))
        surface.blit(label, (sx + 8, sy - 8))

    # Draw X-axis (y = 0)
    x1, y0 = map_to_screen(x_min, 5)
    x2, _ = map_to_screen(x_max, 5)
    pygame.draw.line(surface, (0, 0, 0), (x1, y0), (x2, y0), 2)
    # X ticks
    for x in range(int(x_min), int(x_max) + 1, tick_range):
        sx, sy = map_to_screen(x, max(x_max-x_min,y_max-y_min)//25)
        pygame.draw.line(surface, (0, 0, 0), (sx, sy - 5), (sx, sy + 5), 1)
        label = font.render(str(x), True, (0, 0, 0))
        surface.blit(label, (sx - 10, sy + 8))

# =========================
# f(): bounded random walk
# =========================
prev_x = (x_min + x_max) / 2
prev_y = (y_min + y_max) / 2
Cx = 8

def f(testing = True):
    if not testing:
        target_id = 7
        max_retries = 150
        pos = get_target_position(target_id, max_retries)
        print(pos[0], pos[1])
        return int(pos[0]*100), int(pos[1]*100)
    global prev_x, prev_y, Cx
    if Cx == 0:
        Cx = 8
        x, y = round(random.uniform(x_min, x_max),2), round(random.uniform(y_min, y_max),2)
        prev_x, prev_y = x,y
        return x,y
    Cx -= 1
    dx = random.uniform(-DIFF, DIFF)
    dy = random.uniform(-DIFF, DIFF)

    prev_x = clamp(prev_x + dx, x_min, x_max)
    prev_y = clamp(prev_y + dy, y_min, y_max)
    return round(prev_x,2), round(prev_y,2)

# =========================
# Button class
# =========================
class Button:
    def __init__(self, rect, text, action):
        self.rect = pygame.Rect(rect)
        self.text = text
        self.action = action

    def draw(self, surface, font, mouse_pos):
        color = (200, 200, 200)
        if self.rect.collidepoint(mouse_pos):
            color = (170, 170, 170)

        pygame.draw.rect(surface, color, self.rect, border_radius=6)
        pygame.draw.rect(surface, (0, 0, 0), self.rect, 2, border_radius=6)

        txt = font.render(self.text, True, (0, 0, 0))
        surface.blit(
            txt,
            txt.get_rect(center=self.rect.center)
        )

    def click(self):
        self.action()

# =========================
# Pygame setup
# =========================
pygame.init()
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Coordinate Tracker with UI Buttons")
clock = pygame.time.Clock()
font = pygame.font.SysFont(None, 22)

positions = deque(maxlen= TRACE+1)
permanent_points = []
polygons = []
wall_points = []
walls = []

last_update = 0
running = True

# =========================
# Button actions
# =========================
def store_point():
    if positions:
        permanent_points.append(positions[-1])

def undo_point():
    if permanent_points:
        permanent_points.pop()

def make_polygon():
    if len(permanent_points) >= 3:
        print(f"Poly: {permanent_points}")
        polygons.append(permanent_points.copy())
        permanent_points.clear()

def delete_polygon():
    if polygons:
        polygons.pop()
def store_wall_point():
    if positions:
        wall_points.append(positions[-1])

def undo_wall_point():
    if wall_points:
        wall_points.pop()

def make_wall():
    if len(wall_points) >= 3:
        walls.append(wall_points.copy())
        wall_points.clear()

def delete_wall():
    if walls:
        walls.pop()

# =========================
# Create buttons
# =========================
buttons = [
    Button((10, 10, 150, 40), "STORE POINT", store_point),
    Button((170, 10, 150, 40), "UNDO POINT", undo_point),
    Button((330, 10, 150, 40), "MAKE POLY", make_polygon),
    Button((490, 10, 150, 40), "DEL POLY", delete_polygon),
    Button((10, 60, 150, 40), "STORE WPT", store_wall_point),
    Button((170, 60, 150, 40), "UNDO WPT", undo_wall_point),
    Button((330, 60, 150, 40), "MAKE WALL", make_wall),
    Button((490, 60, 150, 40), "DEL WALL", delete_wall),
]

# =========================
# Main loop
# =========================
while running:
    clock.tick(FPS)
    mouse_pos = pygame.mouse.get_pos()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

        if event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
            for b in buttons:
                if b.rect.collidepoint(event.pos):
                    b.click()

    # Update position
    now = time.time()
    if now - last_update >= UPDATE_INTERVAL:
        last_update = now
        positions.append(f(testing = testing))

    # =========================
    # Drawing
    # =========================
    screen.fill((240, 240, 240))

    # UI bar
    pygame.draw.rect(screen, (220, 220, 220), (0, 0, WIDTH, UI_HEIGHT))
    pygame.draw.line(screen, (0, 0, 0), (0, UI_HEIGHT), (WIDTH, UI_HEIGHT), 2)

    # Buttons
    for b in buttons:
        b.draw(screen, font, mouse_pos)

    # Frame (min/max border)
    pygame.draw.rect(
        screen,
        (0, 0, 0),
        (0, UI_HEIGHT, WIDTH, HEIGHT - UI_HEIGHT),
        2
    )
    offset = (0,0)
    scale = 1
    if walls:
        x_min = min(w[0] for w in walls[0])
        x_max = max(w[0] for w in walls[0])
        y_min = min(w[1] for w in walls[0])
        y_max = max(w[1] for w in walls[0])
    # --- WALLS (BACKGROUND) ---
    for idx, wall in enumerate(walls):
        pts = [map_to_screen(x, y, offset) for x, y in wall]
        pygame.draw.polygon(screen, 'grey' if idx==0 else 'green', pts)
    draw_axes(screen, font, int(max(x_max-x_min,y_max-y_min)/10))
    # --- NORMAL POLYGONS ---
    for poly in polygons:
        pts = [map_to_screen(x, y, offset, scale) for x, y in poly]
        pygame.draw.polygon(screen, (255, 182, 193), pts)

    # --- WALL POINTS ---
    for p in wall_points:
        pygame.draw.circle(screen, (120, 120, 120), map_to_screen(*p, offset, scale), 5)

    # --- NORMAL POINTS ---
    for p in permanent_points:
        pygame.draw.circle(screen, (0, 0, 255), map_to_screen(*p, offset, scale), 6)

    # --- PREVIOUS ---
    for p in list(positions)[:-1]:
        pygame.draw.circle(screen, (0, 180, 0), map_to_screen(*p,offset,scale), 5)

    # --- CURRENT ---
    if positions:
        pygame.draw.circle(screen, (255, 0, 0), map_to_screen(*positions[-1], offset, scale), 7)

    pygame.display.flip()

pygame.quit()
print("All Poly:", polygons)
print("Walls:", walls[0])
print("Safe area:", walls[1])

notsafe_obs_edges = polygons
safe_area = walls[1]
wall_diag = [(x_min,y_min), (x_max,y_max)]