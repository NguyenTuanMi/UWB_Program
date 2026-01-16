import pygame, json, time, math
from typing import List, Dict, Any, Tuple
import tkinter as tk
from tkinter import simpledialog, messagebox
from datetime import datetime

from .constants import *

class Background:
    def __init__(self, image_path):
        try:
            self.original_image = pygame.image.load(image_path)
            print("Background image loaded successfully")
        except Exception as e:
            print(f"Error loading background image: {e}")
            self.original_image = pygame.Surface((SCREEN_WIDTH, SCREEN_HEIGHT))
            self.original_image.fill(BACKGROUND_COLOR)

    def draw(self, surface, scale_factor, screen_coordinates):
        scaled_width = RECT_WIDTH * scale_factor
        scaled_height = RECT_HEIGHT * scale_factor
        scaled_image = pygame.transform.scale(self.original_image, (scaled_width, scaled_height))
        pos_x, pos_y = screen_coordinates(0, RECT_HEIGHT)
        surface.blit(scaled_image, (pos_x, pos_y))

class CoordinateSystem:
    def __init__(self):
        self.scale_factor = INITIAL_SCALE
        self.offset_x = 0
        self.offset_y = 0

    def screen_coordinates(self, x, y):
        screen_x = SCREEN_WIDTH // 2 + (x * self.scale_factor) + self.offset_x
        screen_y = SCREEN_HEIGHT // 2 - (y * self.scale_factor) + self.offset_y
        return int(screen_x), int(screen_y)

    def uwb_coordinates(self, screen_x, screen_y):
        x = (screen_x - SCREEN_WIDTH // 2 - self.offset_x) / self.scale_factor
        y = (SCREEN_HEIGHT // 2 - screen_y + self.offset_y) / self.scale_factor
        return x, y

def load_waypoints():
    """
    Only loads all the waypoints if the LAST waypoint is provided (i.e. distance,angle = 0,0)
    """
    user_input = simpledialog.askstring("Load Waypoints (cancel to clear)", 
                                        f"Enter JSON filename to load (without .json extension) \nDefault: {WAYPOINTS_JSON_DEFAULT}")
        
    filename = WAYPOINTS_JSON_DEFAULT if user_input == "" else f"{user_input}.json"
    
    try:
        with open(filename, 'r') as f:
            data = json.load(f)
            waypoints = data.get("wp", [])
            return [(point['position_cm']['x'] / 100, point['position_cm']['y'] / 100) 
                   for point in waypoints]
    except Exception as e:
        print(f"Error loading waypoints: {e}")
        return []

class Visualization:
    def __init__(self, screen, coord_system):
        self.screen = screen
        self.coord_system = coord_system
        self.position_history = {}
        self.id_colors = {}
        self.persistent_trails = False

    def draw_grid(self):
        start_x, start_y = self.coord_system.uwb_coordinates(0, SCREEN_HEIGHT)
        end_x, end_y = self.coord_system.uwb_coordinates(SCREEN_WIDTH, 0)
        font = pygame.font.Font(None, 20)
        
        for x in range(int(start_x) - 1, int(end_x) + 2):
            screen_x, _ = self.coord_system.screen_coordinates(x, 0)
            if 0 <= screen_x <= SCREEN_WIDTH:
                pygame.draw.line(self.screen, GRID_COLOR, (screen_x, 0), (screen_x, SCREEN_HEIGHT))
                if x != 0:
                    label = font.render(f"{x}", True, LABEL_COLOR)
                    self.screen.blit(label, (screen_x - 10, SCREEN_HEIGHT - 20))

        for y in range(int(start_y) - 1, int(end_y) + 2):
            _, screen_y = self.coord_system.screen_coordinates(0, y)
            if 0 <= screen_y <= SCREEN_HEIGHT:
                pygame.draw.line(self.screen, GRID_COLOR, (0, screen_y), (SCREEN_WIDTH, screen_y))
                if y != 0:
                    label = font.render(f"{y}", True, LABEL_COLOR)
                    self.screen.blit(label, (10, screen_y - 10))

    def draw_rectangle(self):
        points = [
            self.coord_system.screen_coordinates(0, 0),
            self.coord_system.screen_coordinates(RECT_WIDTH, 0),
            self.coord_system.screen_coordinates(RECT_WIDTH, RECT_HEIGHT),
            self.coord_system.screen_coordinates(0, RECT_HEIGHT)
        ]
        pygame.draw.polygon(self.screen, RED, points, 2)

    def draw_loaded_waypoints(self, waypoints):
        if not waypoints:
            return
        
        screen_coords = [self.coord_system.screen_coordinates(x, y) for x, y in waypoints]
        
        for i, point in enumerate(screen_coords):
            pygame.draw.circle(self.screen, BLUE, point, 5)
            if i > 0:
                pygame.draw.line(self.screen, RED, screen_coords[i-1], point, 2)

    # def draw_new_waypoints(self, waypoints):

    #     print(f"NEW WAYPOINTS: {waypoints}")

    #     if not waypoints:
    #         return
        
    #     screen_coords = [self.coord_system.screen_coordinates(point['x'], point['y']) for point in waypoints]
        
    #     for i, point in enumerate(screen_coords):
    #         pygame.draw.circle(self.screen, BLUE, point, 5)
    #         if i > 0:
    #             pygame.draw.line(self.screen, RED, screen_coords[i-1], point, 2)

    def update_positions(self, positions_df):
        current_time = time.time()
        
        for _, row in positions_df.iterrows():
            tag_id = row['id']
            if tag_id not in self.position_history:
                self.position_history[tag_id] = []
                self.id_colors[tag_id] = TAG_COLORS[int(tag_id) % len(TAG_COLORS)]
            
            self.position_history[tag_id].append((current_time, row['x'], row['y']))
            if len(self.position_history[tag_id]) > MAX_HISTORY and not self.persistent_trails:
                self.position_history[tag_id].pop(0)

    def draw_positions(self):
        current_time = time.time()
        font = pygame.font.Font(None, FONT_SIZE)

        for tag_id, positions in self.position_history.items():
            color = self.id_colors[tag_id]
            
            for i in range(len(positions) - 1):
                timestamp, x1, y1 = positions[i]
                _, x2, y2 = positions[i + 1]
                
                alpha = 255 if self.persistent_trails else \
                       int(255 * (1 - (current_time - timestamp) / TRACE_FADE_TIME))
                
                if alpha <= 0:
                    continue
                    
                trail_color = tuple(c * alpha // 255 for c in color)
                start_pos = self.coord_system.screen_coordinates(x1, y1)
                end_pos = self.coord_system.screen_coordinates(x2, y2)
                pygame.draw.line(self.screen, trail_color, start_pos, end_pos, 2)
            
            # Draw current position
            if positions:
                _, x, y = positions[-1]
                screen_x, screen_y = self.coord_system.screen_coordinates(x, y)
                pygame.draw.circle(self.screen, color, (screen_x, screen_y), POINT_RADIUS)
                label = font.render(f"#{int(tag_id)} ({x:.1f}, {y:.1f})", True, color)
                self.screen.blit(label, (screen_x + 10, screen_y - 10))

    def draw_marked_positions(self, marked_positions: List[Dict[str, Any]]):
        """Draw walls, victims, dangers, and waypoints on the screen."""
        
        walls = []
        current_wall_start = None

        # Process walls from wall_start to wall_end
        for point in marked_positions:
            if point["type"] == "wall_start":
                current_wall_start = (point["x"], point["y"])
            elif point["type"] == "wall_end" and current_wall_start:
                walls.append((current_wall_start, (point["x"], point["y"])))
                current_wall_start = None  # Reset for the next segment

        # Draw walls as thick black lines
        for wall in walls:
            start_x, start_y = self.coord_system.screen_coordinates(wall[0][0], wall[0][1])
            end_x, end_y = self.coord_system.screen_coordinates(wall[1][0], wall[1][1])
            pygame.draw.line(self.screen, (100, 100, 200), (start_x, start_y), (end_x, end_y), 5)

        # Draw victims as green triangles
        for victim in (p for p in marked_positions if p["type"] == "victim"):
            screen_x, screen_y = self.coord_system.screen_coordinates(victim["x"], victim["y"])
            pygame.draw.polygon(self.screen, (0, 255, 0), [
                (screen_x, screen_y - 10),
                (screen_x - 10, screen_y + 10),
                (screen_x + 10, screen_y + 10)
            ])

        # Draw dangers as red triangles
        for danger in (p for p in marked_positions if p["type"] == "danger"):
            screen_x, screen_y = self.coord_system.screen_coordinates(danger["x"], danger["y"])
            pygame.draw.polygon(self.screen, (255, 0, 0), [
                (screen_x, screen_y - 10),
                (screen_x - 10, screen_y + 10),
                (screen_x + 10, screen_y + 10)
            ])

        # Draw pillars as black circles
        for pillar in (p for p in marked_positions if p["type"] == "pillar"):
            screen_x, screen_y = self.coord_system.screen_coordinates(pillar["x"], pillar["y"])
            pygame.draw.circle(self.screen, (0, 0, 0), (screen_x, screen_y), 10, 2)

        # Draw waypoints as blue circles with red lines
        # waypoints = [p for p in marked_positions if p["type"] == "waypoint"]
        # self.draw_new_waypoints(waypoints)
        waypoints = [(p['x'], p['y']) for p in marked_positions if p["type"] == "waypoint"]
        self.draw_loaded_waypoints(waypoints)

class RecordingManager:
    def __init__(self):
        self.recording:bool = False
        self.recording_wall:bool = False
        self.recorded_points: List[Dict[str, Any]] = [] # Ordered list of points. Each point is a dictionary with {x: y: z: etc.}
        self.recorded_waypoints: List[Dict[str, Any]] = []  # New list to store waypoints
        self.current_wall_id:int = 0  # To identify different walls
        
    def start_recording(self):
        """Start a new recording session"""
        self.recording:bool = True
        # self.recorded_points = []     # To reset recored_points every time starting recording - NOT NECESSARY
        print("[INFO] Ready to record. Hotkeys: W-Walls (toggle on/off) | P-Pillars | V-Victims")
        
    def stop_recording(self, prev_loaded_points=None):
        """
        Stop recording and save data. Can load existing marked points file, merge with new recorded points, then save separately or overwrite.
        
        """
        self.recording = False
        self.recording_wall = False

        if prev_loaded_points:
            result = self.prompt_merge()
            if result:
                print(f"Currently recorded points: {self.recorded_points}")
                print(f"Prev loaded points: {prev_loaded_points}")
                self.recorded_points.extend(prev_loaded_points)
        
        if self.recorded_points:
            user_input = simpledialog.askstring("Save Marked Positions", 
                                        f"Enter JSON filename to save (without .json extension and UWBViz/ directory) \nDefault: {MARKEDPOINTS_JSON_DEFAULT}")

            if user_input is None:  # User clicked Cancel
                print('Points not saved.')
                return  
            else:
                filename = MARKEDPOINTS_JSON_DEFAULT if user_input == "" else user_input
                full_filename = f"UWBViz/{filename}.json"
            
                # Count pillars, victims, and danger points *before* creating the JSON
                total_pillars = 0
                total_victims = 0
                total_danger = 0
                total_waypoints = 0

                for point in self.recorded_points:  # Assuming self.recorded_points is your list of points
                    point_type = point.get("type")  # Use .get() to avoid KeyError if "type" is missing
                    if point_type == "pillar":  # Adjust the type names as needed
                        total_pillars += 1
                    elif point_type == "victim":
                        total_victims += 1
                    elif point_type == "danger":
                        total_danger += 1
                    elif point_type == "waypoint":
                        total_waypoints += 1

                json_data = {
                        'points': self.recorded_points,
                        'metadata': {   # for user reference only! not used elsewhere
                            'total_points': len(self.recorded_points),
                            'total_wall_segments': self.current_wall_id,
                            'total_pillars': total_pillars,
                            'total_victims': total_victims,
                            'total_danger': total_danger,
                            'total_waypoints': total_waypoints
                        }
                    }
                with open(full_filename, 'w') as f:
                    json.dump(json_data, f, indent=4)
                print(f"[INFO] Recorded positions and waypoints saved to {full_filename}")

                # NEW - save waypoints separately
                if total_waypoints > 0:
                    user_input = simpledialog.askstring("Save Marked Waypoints Separately?", 
                                        f"Enter JSON filename to save waypoints separately (without .json extension and UWBViz/ directory) \nDefault: {MARKEDWAYPOINTS_JSON_DEFAULT}")
                    if user_input is None:  # User clicked Cancel
                        print('Waypoints not saved separately.')
                    else:
                        waypoints_filename = MARKEDWAYPOINTS_JSON_DEFAULT if user_input == "" else user_input
                        waypoints_full_filename = f"UWBViz/{waypoints_filename}.json"

                        converted_json_data = convert_json(json_data)
                    
                        with open(waypoints_full_filename, 'w') as f:
                            json.dump(converted_json_data, f, indent=4)
                        print(f"[INFO] Recorded waypoints saved to {waypoints_full_filename}")
                         
                return filename
            
        # self.recorded_points = []     # This resets recorded_points after saving. Not necessary!
        
    def prompt_merge(self) -> bool:
        """Prompts the user with a yes/no dialog for merging positions."""
        root = tk.Tk()
        root.withdraw()  # Hide the main window

        response = messagebox.askyesno("Merge", "Merge loaded positions with new ones?")
        root.destroy() # Clean up the root window
        return response

    
    def toggle_wall(self, x, y, z):
        """Toggle wall recording state and mark start/end points"""
        if self.recording:
            self.recording_wall = not self.recording_wall
            if self.recording_wall:
                # Start of a new wall
                self.current_wall_id += 1
                self.add_point(x, y, z, point_type='wall_start', wall_id=self.current_wall_id)
                print(f"[INFO] Wall {self.current_wall_id} recording started...")
            else:
                # End of current wall
                self.add_point(x, y, z, point_type='wall_end', wall_id=self.current_wall_id)
                print(f"[INFO] Wall {self.current_wall_id} recording ended")

        else:
            print("[WARNING] Recording not enabled. Press R to enable.")
            
    def add_point(self, x:float, y:float, z:float, point_type:str = ('pillar', 'wall', 'wall_start', 'wall_end', 'victim', 'danger', 'waypoint'), wall_id=None):
        """Add a point to the recording with its type."""
        x, y, z = round(x,2), round(y,2), round(z,2)
        if self.recording:
            point = {
                'x': x,
                'y': y,
                'z': z,
                'type': point_type
            }
            if point_type == 'wall' and wall_id is not None:
                point['wall_id'] = wall_id
            self.recorded_points.append(point)

            print(f"[INFO] {point_type.capitalize()} recorded at {x},{y}")
        else:
            print("[WARNING] Recording not enabled. Press R to enable.")


    def add_waypoint(self, x: float, y: float, z: float):
        """Add a waypoint to the recording."""
        x, y, z = round(x, 2), round(y, 2), round(z, 2)
        if self.recording:
            waypoint = {
                'x': x,
                'y': y,
                'z': z,
                'type': 'waypoint'
            }
            # self.recorded_waypoints.append(waypoint)
            self.recorded_points.append(waypoint)

            print(f"[INFO] Waypoint recorded at {x},{y}")
        else:
            print("[WARNING] Recording not enabled. Press R to enable.")
            
    def remove_last_obj(self):
        if self.recording and not self.recording_wall and self.recorded_points:
            if self.recorded_points[-1].get("type", None) == "wall_end":
                print(f"Removing walls. {self.recorded_points[-2:]}")
                self.recorded_points = self.recorded_points[0:-2]
                self.current_wall_id -= 1
                
            else:
                print(f"Removing point. {self.recorded_points[-1]}")
                self.recorded_points = self.recorded_points[0:-1]
        elif not self.recording:
            print("Unable to undo last object. Ensure recording on!")
        elif self.recording_wall:
            print("Unable to undo last object. Ensure finished recording wall!")
        else:
            print("Unable to undo last object. No points recorded.")

def distance(point1, point2):
    """Calculate Euclidean distance between two points"""
    return ((point1['x'] - point2['x'])**2 + (point1['y'] - point2['y'])**2)**0.5

def convert_json(json_a: Dict) -> Dict:
    """
    Converts marked points json to waypoints json, readable by execute_waypoints.
    
    Args:
        json_a (dict): Marked points json generated by UWBViz, which includes waypoints, together with obstacles, walls, etc. 
    
    Returns:
        dict: Waypoints json readable by execute_waypoints. To be saved as json separately.
    """
    # Extract waypoints from JSON A
    waypoints = [point for point in json_a["points"] if point["type"] == "waypoint"]
    
    # Convert UWB coordinates (meters) to cm
    waypoints_cm = [
        {
            "x": point["x"] * 100,  # Convert meters to cm
            "y": point["y"] * 100   # Convert meters to cm
        }
        for point in waypoints
    ]
    
    # Calculate distance and angle between consecutive waypoints
    wp_list = []
    for i in range(len(waypoints_cm)):
        current_wp = waypoints_cm[i]
        
        if i < len(waypoints_cm) - 1:
            next_wp = waypoints_cm[i + 1]
            
            # Calculate distance in cm
            dx = next_wp["x"] - current_wp["x"]
            dy = next_wp["y"] - current_wp["y"]
            dist_cm = math.hypot(dx, dy)
            
            # Calculate yaw angle in degrees
            if i == 0:  # Initial heading correction
                # Dummy point to simulate initial heading (180 degrees, facing forward)
                dummy_point = (current_wp["x"], current_wp["y"] - 10)  # 10 cm behind the first waypoint
                # Convert current_wp and next_wp to tuples for get_yaw_angle
                current_wp_tuple = (current_wp["x"], current_wp["y"])
                next_wp_tuple = (next_wp["x"], next_wp["y"])
                angle_deg = get_yaw_angle(dummy_point, next_wp_tuple, current_wp_tuple) - 0  # Adjust for initial heading
                if angle_deg > 180:
                    angle_deg -= 360
                elif angle_deg < -180:
                    angle_deg += 360
            else:
                # Calculate yaw angle between previous, current, and next waypoints
                prev_wp = waypoints_cm[i - 1]
                # Convert waypoints to tuples for get_yaw_angle
                prev_wp_tuple = (prev_wp["x"], prev_wp["y"])
                current_wp_tuple = (current_wp["x"], current_wp["y"])
                next_wp_tuple = (next_wp["x"], next_wp["y"])
                angle_deg = get_yaw_angle(prev_wp_tuple, next_wp_tuple, current_wp_tuple)
        else:
            # Final waypoint has dist_cm = 0 and angle_deg = 0
            dist_cm = 0
            angle_deg = 0
        
        # Append to wp_list
        wp_list.append({
            "dist_cm": round(dist_cm),
            "angle_deg": round(angle_deg),
            "position_cm": {
                "x": round(current_wp["x"]),
                "y": round(current_wp["y"])
            }
        })
    
    # Create JSON B structure
    json_b = {
        "README": "JSON stores current waypoint and distance + angle to NEXT waypoint. Final waypoint has dist_cm = angle_deg = 0.",
        "wp": wp_list,
        "competition_notice": "This path was planned in a test area. Scale adjustments needed for 20m x 20m competition field.",
        "datetime": datetime.now().strftime("%m %d %H:%M")  # Add current date and time
    }
    
    return json_b


def get_yaw_angle(pos0: Tuple[float, float], pos1: Tuple[float, float], posref: Tuple[float, float]) -> float:
    """
    Calculate the change in heading required to rotate from the direction vector 'pos0 -> posref'
    to align with the direction vector 'posref -> pos1'.
    
    Args:
        pos0 (Tuple[float, float]): Previous waypoint (x, y).
        pos1 (Tuple[float, float]): Next waypoint (x, y).
        posref (Tuple[float, float]): Current waypoint (x, y).
    
    Returns:
        float: Yaw angle in degrees. Positive for counterclockwise, negative for clockwise.
    """
    # Define the vectors from the points
    v1x, v1y = posref[0] - pos0[0], posref[1] - pos0[1]  # Vector from pos0 to posref
    v2x, v2y = pos1[0] - posref[0], pos1[1] - posref[1]  # Vector from posref to pos1

    # Calculate the dot product and magnitudes of v1 and v2
    dot_product = v1x * v2x + v1y * v2y
    mag_v1 = math.hypot(v1x, v1y)
    mag_v2 = math.hypot(v2x, v2y)

    # Ensure there's no division by zero if any vector has zero magnitude
    if mag_v1 * mag_v2 == 0:
        return 0.0
    
    # Calculate the angle's magnitude (in radians) using the dot product
    cos_theta = dot_product / (mag_v1 * mag_v2)
    cos_theta = max(-1.0, min(1.0, cos_theta))  # Clamp to avoid floating-point errors
    angle_rad = math.acos(cos_theta)
    angle_deg = math.degrees(angle_rad)

    # Use the cross product to determine the sign (direction) of the angle
    cross_product = v1x * v2y - v1y * v2x
    if cross_product < 0:  # Flip the sign to match CCW = +ve, CW = -ve
        angle_deg = -angle_deg

    return angle_deg

class Button:
    def __init__(self, x, y, width, height, text, font, color, hover_color, text_color):
        self.rect = pygame.Rect(x, y, width, height)
        self.text = text
        self.font = font
        self.color = color
        self.hover_color = hover_color
        self.text_color = text_color
        self.hovered = False

    def draw(self, screen):
        # Change color if hovered
        if self.hovered:
            pygame.draw.rect(screen, self.hover_color, self.rect)
        else:
            pygame.draw.rect(screen, self.color, self.rect)

        # Render the text
        text_surface = self.font.render(self.text, True, self.text_color)
        text_rect = text_surface.get_rect(center=self.rect.center)
        screen.blit(text_surface, text_rect)

    def check_hover(self, mouse_pos):
        self.hovered = self.rect.collidepoint(mouse_pos)

    def is_clicked(self, mouse_pos):
        return self.hovered and self.rect.collidepoint(mouse_pos)
    
key_to_number = {
    pygame.K_1: 1,
    pygame.K_2: 2,
    pygame.K_3: 3,
    pygame.K_4: 4,
    pygame.K_5: 5,
    pygame.K_6: 6,
    pygame.K_7: 7,
    pygame.K_8: 8,
    pygame.K_9: 9,
    pygame.K_0: 0
}