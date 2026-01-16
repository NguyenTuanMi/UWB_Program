import pygame
import sys
import time
import threading
import json
import copy
from pathlib import Path

from .constants import *
from .utils import *
import pandas as pd

import tkinter as tk
from tkinter import simpledialog, ttk

"""
    python -m UWBViz.main
    NOTE: To apply UWB offset (i.e. when UWB origin not at map 0,0), see UWB_ReadUDP
"""

# Add workspace root to sys.path (9 Jan: Works but might need a better solution)
workspace_root = Path(__file__).resolve().parent.parent
sys.path.append(str(workspace_root))
from UWB_Manipulation.UWB_Reader import get_all_positions

# Create a Tkinter root window (but don't show it)
root = tk.Tk()
root.withdraw()  # Hide the main Tkinter window

user_input = ""  # Store the user's input

class UWBVisualization:
    def __init__(self):
        pygame.init()
        self.screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT), pygame.RESIZABLE)
        pygame.display.set_caption("UWB Position Visualization")
        
        self.coord_system = CoordinateSystem()
        self.visualization = Visualization(self.screen, self.coord_system)
        self.background = Background(BGPIC)

        font = pygame.font.Font(None, 36)

        self.button1 = Button(100, 100, 200, 50, "Button 1", font, GREEN, BLUE, BLACK)
        self.button2 = Button(100, 200, 200, 50, "Button 2", font, GREEN, BLUE, BLACK)
        
        self.latest_positions_data = None       # Stores a local copy of ALL positions df
        self.data_lock = threading.Lock()
        self.controls_enabled = True
        self.panning = False
        self.last_mouse_pos = None
        self.waypoints = []
        self.loaded_marked_pos = None
        self.loaded_marked_pos_filename: str = None  

        self.tag_registered:int = 0

        self.last_cmdline: str = None        # Disabled 15 Feb - too complex to implement printing cmdline on pygame
        
        # Mouse simulation variables
        self.mouse_simulation = False
        self.simulated_tags = {
            0: {'x': 0, 'y': 0, 'z': 0},
        }
        
        # Recording manager
        self.recording_manager = RecordingManager()

        # Thread control
        self.data_event = threading.Event()
        self.data_event.set()

        # Start data collection thread
        self.data_thread = threading.Thread(target=self.collect_data, daemon=True)
        self.data_thread.start()

    def load_marked_positions(self, filename=None):
        """Load marked positions from JSON file. Do not include .json extension and UWBViz/ directory"""

        # if not filename:
        #     user_input = simpledialog.askstring("Load Marked Positions", 
        #                                     f"Enter JSON filename to load (without .json extension) \nDefault: {MARKEDPOINTS_JSON_DEFAULT}")
            
        #     filename = MARKEDPOINTS_JSON_DEFAULT if user_input == "" else f"{user_input}.json"
        if filename:
            full_filename = f"UWBViz/{filename}.json"
            print(f"Full filename: {full_filename}")
        else:
            print("No filename provided. Not loading.")
            return

        try:
            with open(full_filename, 'r') as f:
                self.loaded_marked_pos = json.load(f)["points"]
                self.loaded_marked_pos_filename = filename
            print("[INFO] Marked positions loaded successfully.")
        except FileNotFoundError:
            print("[INFO] No saved marked positions found.")
        except Exception as e:
            print(f"[ERROR] Could not load marked positions: {e}")

    def marked_points_dialog(self, parent_window=None):

        # Create a Tkinter window for the dropdown
        dialog_window = tk.Toplevel(parent_window)
        dialog_window.title("Marked Points Manager")
        dialog_width = 400
        dialog_height = 150  # Reduced height since we have fewer buttons
        dialog_window.geometry(f"{dialog_width}x{dialog_height}")
        
        # Force to be on top and take focus
        dialog_window.attributes("-topmost", True)
        
        # Center dialog on screen
        dialog_window.update_idletasks()
        screen_width = dialog_window.winfo_screenwidth()
        screen_height = dialog_window.winfo_screenheight()
        x = (screen_width - dialog_window.winfo_width()) // 2
        y = (screen_height - dialog_window.winfo_height()) // 2
        dialog_window.geometry(f"+{x}+{y}")
        
        # Get list of JSON files in UWBViz/ directory
        uwbviz_dir = "UWBViz/"
        json_files = []
        try:
            json_files = [f.replace('.json', '') for f in os.listdir(uwbviz_dir) if f.endswith('.json')]
        except FileNotFoundError:
            print(f"Warning: Directory {uwbviz_dir} not found")
        
        # Ensure default is in the list
        if MARKEDPOINTS_JSON_DEFAULT not in json_files:
            json_files.insert(0, MARKEDPOINTS_JSON_DEFAULT)
        else:
            # Move default to the first position
            json_files.remove(MARKEDPOINTS_JSON_DEFAULT)
            json_files.insert(0, MARKEDPOINTS_JSON_DEFAULT)
        
        # Create label
        tk.Label(dialog_window, text=f"Select JSON file to load:\nLast loaded: {self.loaded_marked_pos_filename}").pack(pady=10)
        
        # Create combobox (dropdown)
        filename_var = tk.StringVar(value=MARKEDPOINTS_JSON_DEFAULT)
        filename_combo = ttk.Combobox(dialog_window, textvariable=filename_var, values=json_files, width=30)
        filename_combo.pack(pady=5)
        filename_combo.current(0)  # Select default option
        
        # Make combobox read-only for load
        filename_combo.configure(state="readonly")
        
        # Variables to store result
        result = {"filename": None, "action": None}
        
        def on_load():
            result["filename"] = filename_var.get()
            result["action"] = "load"
            dialog_window.destroy()
        
        def on_cancel():
            dialog_window.destroy()

        def on_clear():
            """
            Resets the marked points to None
            """
            self.loaded_marked_pos = None
            self.recording_manager.recorded_points = []
            self.recording_manager.current_wall_id = 0
            self.loaded_marked_pos_filename = None
            dialog_window.destroy()
        
        # Handle Enter key press to load
        def on_enter(event):
            on_load()
        
        # Handle Escape key press to cancel
        def on_escape(event):
            on_cancel()
        
        # Bind Enter and Escape keys to the window
        dialog_window.bind('<Return>', on_enter)
        dialog_window.bind('<Escape>', on_escape)
        
        # Button frame
        button_frame = tk.Frame(dialog_window)
        button_frame.pack(pady=15)
        
        # Load and Cancel buttons
        load_button = tk.Button(button_frame, text="Load", command=on_load)
        cancel_button = tk.Button(button_frame, text="Cancel", command=on_cancel)
        clear_button = tk.Button(button_frame, text="Clear", command=on_clear)
        
        # Load button has initial focus
        load_button.config(relief=tk.SUNKEN)
        
        # Pack buttons
        load_button.pack(side=tk.LEFT, padx=10)
        cancel_button.pack(side=tk.LEFT, padx=10)
        clear_button.pack(side=tk.LEFT, padx=10)
        
        # Ensure the window takes focus
        dialog_window.after(100, lambda: dialog_window.focus_force())
        filename_combo.focus_set()
        
        # Make window modal (blocks until closed)
        dialog_window.grab_set()
        dialog_window.wait_window()
        
        # Process result
        if result["action"] == "load":
            # Call load_marked_positions function with the selected filename
            self.load_marked_positions(result["filename"])
            print(f"Loaded marked points from {result['filename']}.json")
            return True
        
        return False

    def collect_data(self):
        while True:
            self.data_event.wait()  # Block if the event is cleared (paused)
            
            if self.mouse_simulation:       # IMPT: This blocks all other real UWB readings!
                # Create DataFrame from mouse position
                mouse_x, mouse_y = pygame.mouse.get_pos()
                uwb_x, uwb_y = self.coord_system.uwb_coordinates(mouse_x, mouse_y)
                self.simulated_tags[0]['x'] = uwb_x
                self.simulated_tags[0]['y'] = uwb_y
                
                # Convert simulated data to DataFrame
                rows = []
                for tag_id, pos in self.simulated_tags.items():
                    rows.append({
                        'id': tag_id,
                        'role': 0,
                        'x': pos['x'],
                        'y': pos['y'],
                        'z': pos['z'],
                        'timestamp': time.time()
                    })
                df = pd.DataFrame(rows)
                
                with self.data_lock:
                    self.latest_positions_data = df
            else:
                try:
                    df = get_all_positions()
                    if not df.empty:
                        with self.data_lock:
                            self.latest_positions_data = df.groupby('id').last().reset_index()
                except Exception as e:
                    print(f"Error reading data: {e}")
            
            time.sleep(0.01)

    def handle_events(self):
        for event in pygame.event.get():
            if event.type == pygame.QUIT or (
                event.type == pygame.KEYDOWN and 
                event.key == pygame.K_ESCAPE
            ):
                if self.recording_manager.recording:
                    filename = self.recording_manager.stop_recording(self.loaded_marked_pos)
                    self.load_marked_positions(filename)
                return False
                    
            elif event.type == pygame.VIDEORESIZE:
                # Handle window resize
                self.screen = pygame.display.set_mode((event.w, event.h), pygame.RESIZABLE)
                self.visualization.screen = self.screen  # Update visualization's screen reference
                    
            elif event.type == pygame.MOUSEWHEEL and self.controls_enabled:
                self.handle_zoom(event.y)
                    
            elif event.type == pygame.MOUSEBUTTONDOWN and event.button == 1:
                if self.controls_enabled:
                    self.panning = True
                    self.last_mouse_pos = pygame.mouse.get_pos()

                # Check for button clicks
                mouse_pos = pygame.mouse.get_pos()
                if self.button1.is_clicked(mouse_pos):
                    print("Button 1 clicked!")
                    # Trigger your event here
                if self.button2.is_clicked(mouse_pos):
                    print("Button 2 clicked!")
                    # Trigger your event here
                        
            elif event.type == pygame.MOUSEBUTTONUP and event.button == 1:
                self.panning = False
                    
            elif event.type == pygame.MOUSEMOTION and self.panning and self.controls_enabled:
                current_pos = pygame.mouse.get_pos()
                self.coord_system.offset_x += current_pos[0] - self.last_mouse_pos[0]
                self.coord_system.offset_y += current_pos[1] - self.last_mouse_pos[1]
                self.last_mouse_pos = current_pos
                    
            elif event.type == pygame.KEYDOWN:
                self.handle_keypress(event.key)

        # Update hover state for buttons
        mouse_pos = pygame.mouse.get_pos()
        self.button1.check_hover(mouse_pos)
        self.button2.check_hover(mouse_pos)

        return True

    def handle_zoom(self, y):
        mouse_x, mouse_y = pygame.mouse.get_pos()
        pre_zoom_uwb_x, pre_zoom_uwb_y = self.coord_system.uwb_coordinates(mouse_x, mouse_y)
        old_scale = self.coord_system.scale_factor
        self.coord_system.scale_factor = max(10, min(INITIAL_SCALE * 10, self.coord_system.scale_factor + y * 5))
        new_screen_x, new_screen_y = self.coord_system.screen_coordinates(pre_zoom_uwb_x, pre_zoom_uwb_y)
        self.coord_system.offset_x += (mouse_x - new_screen_x)
        self.coord_system.offset_y += (mouse_y - new_screen_y)

    def handle_keypress(self, key):
        current_pos = None
        if self.mouse_simulation:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            uwb_x, uwb_y = self.coord_system.uwb_coordinates(mouse_x, mouse_y)
            current_pos = {'x': uwb_x, 'y': uwb_y, 'z': 0}
        elif self.latest_positions_data is not None and not self.latest_positions_data.empty:
            # print(f"DEBUG 11 MAR: latest_positions_data = {self.latest_positions_data}")
            if self.tag_registered != 0:
                last_pos = self.latest_positions_data.loc[self.latest_positions_data['id'] == self.tag_registered]
            else:
                last_pos = self.latest_positions_data.iloc[-1]      # caa 11 Mar : currently takes the LARGEST tag number (to work)
            
            current_pos = {'x': last_pos['x'], 'y': last_pos['y'], 'z': last_pos.get('z', 0)}

        if key == pygame.K_SPACE:
            self.visualization.persistent_trails = not self.visualization.persistent_trails
            if not self.visualization.persistent_trails:
                self.visualization.position_history.clear()
        elif key == pygame.K_RETURN:
            self.controls_enabled = not self.controls_enabled
            if not self.controls_enabled:
                self.panning = False
        elif key == pygame.K_l:
            self.pause_data_thread()
            self.waypoints = load_waypoints()
            self.resume_data_thread()
        elif key == pygame.K_u:  # Toggle mouse simulation
            self.mouse_simulation = not self.mouse_simulation
            if self.mouse_simulation:
                print("[INFO] Mouse simulation enabled")
            else:
                print("[INFO] Mouse simulation disabled")
        elif key == pygame.K_r:  # Toggle recording
            if not self.recording_manager.recording:
                self.recording_manager.start_recording()
            else:
                filename = self.recording_manager.stop_recording(self.loaded_marked_pos)
                self.load_marked_positions(filename)
        elif key == pygame.K_w and current_pos:  # Toggle wall recording
            self.recording_manager.toggle_wall(current_pos['x'], current_pos['y'], current_pos['z'])
        elif key == pygame.K_p and current_pos:  # Add pillar obstacle
            self.recording_manager.add_point(current_pos['x'], current_pos['y'], current_pos['z'], 'pillar')
        elif key == pygame.K_v and current_pos:
            self.recording_manager.add_point(current_pos['x'], current_pos['y'], current_pos['z'], 'victim')
        elif key == pygame.K_d and current_pos:
            self.recording_manager.add_point(current_pos['x'], current_pos['y'], current_pos['z'], 'danger')
        elif key == pygame.K_q and current_pos and self.recording_manager.recording:    # Add waypoint
            self.recording_manager.add_point(current_pos['x'], current_pos['y'], current_pos['z'], 'waypoint')
        elif key == pygame.K_z:
            self.recording_manager.remove_last_obj()
        elif key == pygame.K_m:
            result = self.marked_points_dialog()
            if result:
                print("Marked points loaded successfully")
            else:
                print("Marked points not loaded")
        elif key in (pygame.K_1, pygame.K_2, pygame.K_3, pygame.K_4, pygame.K_5, pygame.K_6, pygame.K_7, pygame.K_8, pygame.K_9, pygame.K_0) and not self.recording_manager.recording:
            number_pressed = key_to_number.get(key)
            self.tag_registered = int(number_pressed)
            print(f"Registering tag number: {number_pressed}")

    def pause_data_thread(self):
        print("[INFO] Pausing data collection thread. Please type input in the terminal.")
        self.data_event.clear()  # Pause the thread

    def resume_data_thread(self):
        print("[INFO] Resuming data collection thread.")
        self.data_event.set()  # Resume the thread

    def update(self):
        with self.data_lock:
            current_positions = copy.deepcopy(self.latest_positions_data)
        
        if current_positions is not None:
            self.visualization.update_positions(current_positions)

    def draw(self):
        self.screen.fill(BACKGROUND_COLOR)
        
        # Get current screen dimensions
        screen_width, screen_height = self.screen.get_size()
        
        self.background.draw(self.screen, self.coord_system.scale_factor, self.coord_system.screen_coordinates)
        self.visualization.draw_grid()
        self.visualization.draw_rectangle()
        self.visualization.draw_positions()  # Draw UWB positions

        self.visualization.draw_loaded_waypoints(self.waypoints)  # Draw pre-loaded waypoints
        
        if self.loaded_marked_pos:  # Draws pre-loaded positions
            self.visualization.draw_marked_positions(self.loaded_marked_pos)

        if self.recording_manager.recorded_points:  # Draw marked positions and newly marked waypoints
            self.visualization.draw_marked_positions(self.recording_manager.recorded_points)

        # Draw white boxes behind text for better readability
        if self.recording_manager.recording:
            font = pygame.font.Font(None, 36)
            status = f"Recording Wall {self.recording_manager.current_wall_id}..." if self.recording_manager.recording_wall else "Ready to Record (press R to stop recording)"
            self.last_cmdline = status
            text = font.render(status, True, (255, 0, 0))
            
            # Calculate the size of the text and create a white box behind it
            text_width, text_height = font.size(status)
            pygame.draw.rect(self.screen, (255, 255, 255), (25, 5, text_width + 10, text_height + 10))  # White box
            self.screen.blit(text, (30, 10))  # Blit text on top of the white box

        # Instructions
        font = pygame.font.Font(None, 25)
        instructions = "ESC: exit, SPACE: toggle trails, ENTER: toggle pan & zoom, L: load waypoints"
        instructions2 = "R: toggle recording, W: toggle walls, V/D/P: mark Victims/Dangers/Pillars, M: load markers"
        
        # Render instructions text
        text = font.render(instructions, True, (0, 0, 0))
        text2 = font.render(instructions2, True, (0, 0, 0))
        
        # Calculate the size of the instructions text and create white boxes behind them
        text_width, text_height = font.size(instructions)
        text2_width, text2_height = font.size(instructions2)
        
        # Draw white boxes behind the instructions
        pygame.draw.rect(self.screen, (255, 255, 255), (25, 35, text_width + 10, text_height + 10))  # White box for instructions
        pygame.draw.rect(self.screen, (255, 255, 255), (25, 65, text2_width + 10, text2_height + 10))  # White box for instructions2
        
        # Blit instructions text on top of the white boxes
        self.screen.blit(text, (30, 40))
        self.screen.blit(text2, (30, 70))

        # Draw buttons (DISABLED 13 MAR)
        # self.button1.draw(self.screen)
        # self.button2.draw(self.screen)

        pygame.display.update()

    def run(self):
        clock = pygame.time.Clock()
        print("[INFO] Press ESC to exit, SPACE for trails, ENTER for controls, L to load waypoints")
        print("[INFO] V/D/P to mark Victims/Dangers/Pillars, M to load markers")
        
        running = True
        while running:
            running = self.handle_events()
            self.update()
            self.draw()
            clock.tick(30)
        
        pygame.quit()
        sys.exit()

if __name__ == "__main__":
    app = UWBVisualization()
    app.run()