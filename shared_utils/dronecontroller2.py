import torch
import cv2
import time
from cv2 import aruco
import numpy as np
import math
import threading
from threading import Lock, Event
import logging  # in decreasing log level: debug > info > warning > error > critical
from typing import List, Dict


from .customtello import CustomTello, MockTello
from swarmserver.swarmserverclient import MarkerClient
from .shared_utils import *

"""
17 Feb LATEST Testing - V2 for Simultaneous takeoff and integrating ALL controllers
11 Mar Stable - Moved takeoff_simul to MarkerClient; Midas into centre subsections
"""

class DroneController:
    """
    No takeoff / flying / landing commands takes place here. (caa 17 Feb)
    """
    def __init__(self, network_config, drone_id, laptop_only = False, load_midas = True, imshow = True):
        # Initialize Tello
        logging.debug(f"laptop_only = {laptop_only}")
        self.drone = MockTello() if laptop_only else CustomTello(network_config)
        
        self.imshow = imshow
        self.laptop_only = laptop_only
        
        if load_midas:
            # Initialize MiDaS model
            self.model_type = "MiDaS_small"
            self.midas = torch.hub.load("intel-isl/MiDaS", self.model_type)
            self.device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")
            self.midas.to(self.device)
            self.midas.eval()
            
            # Load MiDaS transform
            midas_transforms = torch.hub.load("intel-isl/MiDaS", "transforms")
            self.transform = midas_transforms.small_transform

        # Initialize Swarm Client
        self.marker_client = MarkerClient(drone_id = drone_id, land_callback=self.handle_land_signal)

        # Video Stream Properties        
        self.current_frame = None
        self.display_frame = None
        self.frame_lock = Lock()
        self.stream_thread = None
        self.stop_event = Event()

        ## COMMENT OUT BELOW FOR TESTING MULTIPLE DRONES USING NO_FLY = FALSE ON LAPTOP ONLY (USEFUL FOR TESTING CLIENTS REMOTELY), BUT ALSO NEED TO COMMENT OUT ALL OTHER GET.BATTERY() ETC. ------------------------------------

        self.drone.connect()
        logging.info(f"Start Battery Level: {self.drone.get_battery()}%")

        if not self.laptop_only:
            start_time = time.time()
            self.drone.set_video_resolution(self.drone.RESOLUTION_480P)     # IMPT: Default 720P - need to use correct calibration params if set to 480P 
            self.drone.set_video_fps(self.drone.FPS_15)
            self.drone.set_video_bitrate(self.drone.BITRATE_3MBPS)
            duration = time.time() - start_time
            logging.debug(f"Video stream parameters set in {duration:.1f}s")
        
        ## COMMENT OUT ABOVE FOR TESTING. ALTERNATIVELY, USE MULTIPLE LAPTOP_ONLY = TRUE ----------------------------------

        # Color depth map
        self.depth_map_colors = {}
        
        # Controller state
        self.frame = None
        self.frame_lock = Lock()
        self.distance = None        # 29 Jan Gab: This is the 3D distance - decently accurate
        self.distance_lock = Lock()
        self.is_running = True
        self.marker_detected = False
        self.markernum_lockedon:int = None
        self.is_centered = False
        self.movement_completed = False
        self.valid_ids = set(range(1, 9))   # set because doesn't change
        self.invalid_ids = []               # detected/landed by other drones, updated by MarkerClient
        self.danger_ids = set(range(9, 15)) # set because doesn't change
        self.exit_ids = set(range(50, 100)) # set because doesn't change; for drone to avoid exiting search area
        self.exit_detected = False
        self.exit_distance_3D = None
        self.marker_positions = {}
        self.valid_marker_info:dict = {}    # stores the data of ONE valid, locked-on marker
         
        # Navigation parameters
        self.drone_id = drone_id
        self.move_speed = 20    # usually 20
        self.yaw_speed = 50     # usually 50
        self.forward_tof_dist = 0 # to be updated by subthread
        self.forward_tof_lock = Lock()
        self.target_yaw = None      # TESTING END-JAN - for exit marker (still testing caa 26 Feb)

        # Danger offset parameters
        self.shortest_danger_distance:float = float("inf")  
        self.nearest_danger_id:int = None
        self.nearest_danger_data:dict = None  # stores the data of ONE nearest danger marker closest to valid_marker_info
        self.danger_offset:tuple[int] = (0,0,0)
        self.no_danger_count:int = 0
  
    def setup_stream(self):
        start_time = time.time()
        self.drone.streamon()
        self.frame_reader = self.drone.get_frame_read()
        self.start_video_stream(imshow=self.imshow)  # IMPT: DO NOT call cv2.imshow in code - Comment out to deactivate, otherwise will cause lag!
        logging.info(f"Initializing frame reader... imshow = {self.imshow}")
        time_taken = time.time() - start_time
        logging.info(f"setup_stream completed in {time_taken:.2f}s")
        time.sleep(2)
    
    def handle_land_signal(self):
        """
        Callback function to handle the land signal. 
        Does not work during execute_waypoints!
        """
        logging.info(f"Land signal received for Tello {self.drone_id}. Shutting down...")
        self.shutdown()
        self.marker_client.land_signal = False  # Reset the flag

    def start_video_stream(self, imshow:bool = True):
        """Start the video streaming in a separate thread"""
        self.stop_event.clear()
        self.stream_thread = threading.Thread(target=self._stream_video, args=(imshow,))
        self.stream_thread.daemon = True
        self.stream_thread.start()
        
    def stop_video_stream(self):
        """Stop the video streaming thread. Called in class function shutdown()"""
        self.stop_event.set()
        try:
            if self.stream_thread and self.stream_thread.is_alive():
                self.stream_thread.join(timeout=2)
        except Exception as e:
            logging.error(f"Error in stopping video stream: {e}")

    def get_current_frame(self):
        """Thread-safe method to get the latest frame. External method."""
        with self.frame_lock:
            return self.current_frame.copy() if self.current_frame is not None else None
        
    def get_display_frame(self):
        """Thread-safe method to get the display frame. Internal method."""
        with self.frame_lock:
            return self.display_frame.copy() if self.display_frame is not None else None
        
    def set_display_frame(self, frame):
        """Thread-safe method to set the display frame. External method."""
        with self.frame_lock:
            self.display_frame = frame.copy() if frame is not None else None
    
    def _stream_video(self, imshow: bool = True):
        """Video streaming thread function.
        This function now only updates the current frame without calling cv2.imshow.
        Display is handled separately in the main thread.
        """
        while not self.stop_event.is_set():
            try:
                # Capture new frame
                frame = capture_frame(self.frame_reader)
                if frame is None:
                    continue
                        
                # Store frame thread-safely
                with self.frame_lock:
                    self.current_frame = frame.copy()
                    
            except Exception as e:
                logging.error(f"Error in video stream: {e}")
                time.sleep(0.1)
        logging.info("_stream_video exited.")        
        self.shutdown()

    def handle_key_command(self, key):
        """Handle keyboard commands - can be overridden by subclasses"""
        pass

    def generate_color_depth_map(self, frame):
        """Process frame through MiDaS to get depth map"""
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        input_batch = self.transform(frame_rgb).to(self.device)
        
        with torch.no_grad():
            prediction = self.midas(input_batch)
            
        depth_map = prediction.squeeze().cpu().numpy()
        depth_map = cv2.normalize(depth_map, None, 0, 1, norm_type=cv2.NORM_MINMAX)
        return cv2.applyColorMap((depth_map * 255).astype(np.uint8), cv2.COLORMAP_JET)
        
    def process_depth_color_map(self, depth_colormap):
        """
        :param depth_colormap: the actual frame of the depth color map
        :return: None
        Splits the depth map into a 3x3 grid and performs two levels of analysis:
        - Overall middle row analysis (middle_left, middle_center, middle_right).
        - Further subdivision of the middle_center region into three columns.
        Updates class attributes.
        """
        h, w = depth_colormap.shape[:2]

        # Define the 3x3 grid regions
        top_left     = depth_colormap[:h//3, :w//3]
        top_center   = depth_colormap[:h//3, w//3:2*w//3]
        top_right    = depth_colormap[:h//3, 2*w//3:]
        
        middle_left  = depth_colormap[h//3:2*h//3, :w//3]
        middle_center= depth_colormap[h//3:2*h//3, w//3:2*w//3]
        middle_right = depth_colormap[h//3:2*h//3, 2*w//3:]
        
        bottom_left  = depth_colormap[2*h//3:, :w//3]
        bottom_center= depth_colormap[2*h//3:, w//3:2*w//3]
        bottom_right = depth_colormap[2*h//3:, 2*w//3:]
        
        # Draw the outer grid lines (green)
        cv2.line(depth_colormap, (w//3, 0), (w//3, h), (0, 255, 0), 2)
        cv2.line(depth_colormap, (2*w//3, 0), (2*w//3, h), (0, 255, 0), 2)
        cv2.line(depth_colormap, (0, h//3), (w, h//3), (0, 255, 0), 2)
        cv2.line(depth_colormap, (0, 2*h//3), (w, 2*h//3), (0, 255, 0), 2)
        
        # Analyze colors in the middle row regions
        red_middle_left = np.sum((middle_left[:, :, 2] > 150) & (middle_left[:, :, 0] < 50))
        blue_middle_left = np.sum((middle_left[:, :, 0] > 150) & (middle_left[:, :, 2] < 50))
        
        red_middle_center = np.sum((middle_center[:, :, 2] > 150) & (middle_center[:, :, 0] < 50))
        blue_middle_center = np.sum((middle_center[:, :, 0] > 150) & (middle_center[:, :, 2] < 50))
        
        red_middle_right = np.sum((middle_right[:, :, 2] > 150) & (middle_right[:, :, 0] < 50))
        blue_middle_right = np.sum((middle_right[:, :, 0] > 150) & (middle_right[:, :, 2] < 50))
        
        # Further split the middle_center region into 3 columns for detailed analysis
        mc_row_start = h//3
        mc_row_end = 2*h//3
        mc_col_start = w//3
        mc_col_end = 2*w//3
        mc_width = mc_col_end - mc_col_start  # roughly w//3
        sub_width = mc_width // 3             # width of each subdivided region
        
        middle_center_left = depth_colormap[mc_row_start:mc_row_end, mc_col_start: mc_col_start + sub_width]
        middle_center_center = depth_colormap[mc_row_start:mc_row_end, mc_col_start + sub_width: mc_col_start + 2*sub_width]
        middle_center_right = depth_colormap[mc_row_start:mc_row_end, mc_col_start + 2*sub_width: mc_col_end]
        
        # Draw extra vertical lines (blue) within the middle_center region to delineate subdivisions
        cv2.line(depth_colormap, (mc_col_start + sub_width, mc_row_start), (mc_col_start + sub_width, mc_row_end), (255, 0, 0), 2)
        cv2.line(depth_colormap, (mc_col_start + 2*sub_width, mc_row_start), (mc_col_start + 2*sub_width, mc_row_end), (255, 0, 0), 2)
        
        # Analyze colors in the subdivided middle_center subregions
        red_mc_left = np.sum((middle_center_left[:, :, 2] > 150) & (middle_center_left[:, :, 0] < 50))
        nonblue_mc_left = np.sum(~((middle_center_left[:, :, 0] > 150) & (middle_center_left[:, :, 2] < 50)))
        blue_mc_left = np.sum((middle_center_left[:, :, 0] > 150) & (middle_center_left[:, :, 2] < 50))
        
        red_mc_center = np.sum((middle_center_center[:, :, 2] > 150) & (middle_center_center[:, :, 0] < 50))
        nonblue_mc_center = np.sum(~((middle_center_center[:, :, 0] > 150) & (middle_center_center[:, :, 2] < 50)))
        blue_mc_center = np.sum((middle_center_center[:, :, 0] > 150) & (middle_center_center[:, :, 2] < 50))
        
        red_mc_right = np.sum((middle_center_right[:, :, 2] > 150) & (middle_center_right[:, :, 0] < 50))
        nonblue_mc_right = np.sum(~((middle_center_right[:, :, 0] > 150) & (middle_center_right[:, :, 2] < 50)))
        blue_mc_right = np.sum((middle_center_right[:, :, 0] > 150) & (middle_center_right[:, :, 2] < 50))
        
        # Update class attributes with both levels of analysis
        self.depth_map_colors["middle_row"] = {
            "middle_left": {"red": red_middle_left, "blue": blue_middle_left},
            "middle_center": {"red": red_middle_center, "blue": blue_middle_center},
            "middle_right": {"red": red_middle_right, "blue": blue_middle_right}
        }
        
        self.depth_map_colors["middle_center_split"] = {
            "left": {"red": red_mc_left, "blue": blue_mc_left, "nonblue": nonblue_mc_left},
            "center": {"red": red_mc_center, "blue": blue_mc_center, "nonblue": nonblue_mc_center},
            "right": {"red": red_mc_right, "blue": blue_mc_right, "nonblue": nonblue_mc_right}
        }

    def get_distance(self):
        with self.distance_lock:
            return self.distance
            
    def set_distance(self, distance):
        with self.distance_lock:
            self.distance = distance

    def get_tof_distance(self):
        with self.forward_tof_lock:
            return self.forward_tof_dist
        
    def get_tof_distances_list(self, list_length:int, interval_s:int = 0.6) -> List:
        """
        Returns a list of ToF readings; Useful for taking average
        Does not count 8888 readings
        Args:
            list_length: length of list to return
            interval_s: duration between ToF readings. Should exceed _tof_thread period of 0.5s.
        """
        tof_dist_list = []
        i = 0
        while i < list_length:
            with self.forward_tof_lock:
                current_dist = self.forward_tof_dist
                if current_dist == 8888:
                    # Repeat the reading if the distance is 8888
                    continue
                tof_dist_list.append(current_dist)
                i += 1  # Only increment the counter if a valid reading is added
            time.sleep(interval_s)
        return tof_dist_list
    
    def tof_check_clear(self, tof_dist_list: list, clear_threshold_cm: int = 2000, clear_ratio: float = 0.6) -> bool:
        """
        Checks a list of ToF distances to determine if the path is clear of obstacles.

        Args:
            tof_dist_list: List of ToF readings in cm; Assumes no invalid readings (e.g., 8888).
            clear_threshold_cm: Distance threshold in cm below which an obstacle is considered detected. Maximum ToF sensing ~ 1.2m
            clear_ratio: Minimum ratio of readings above the threshold to consider the path clear.

        Returns:
            bool: True if the path is clear (ratio of clear readings >= clear_ratio), False otherwise.
        """
        if not tof_dist_list:
            # Handle empty list case
            return False

        # Count the number of readings above the clear threshold
        clear_readings = sum(1 for reading in tof_dist_list if reading >= clear_threshold_cm)

        # Calculate the ratio of clear readings
        calculated_ratio = clear_readings / len(tof_dist_list)

        # Determine if the path is clear based on the ratio
        return calculated_ratio >= clear_ratio
            

    def _tof_thread(self, period_s:float = 0.5):
        """Video streaming thread function"""
        logging.info("_tof_thread started.")
        while not self.stop_event.is_set():
            time.sleep(period_s)
            try:
                with self.forward_tof_lock:
                    self.forward_tof_dist = self.drone.get_ext_tof()
            except Exception as e:
                logging.error(f"Error in ToF thread: {e}")
                time.sleep(0.1)
        logging.info("_tof_thread exited.")

    def start_tof_thread(self):
        """Start the forward ToF reading in a separate thread"""
        self.stop_event.clear()
        self.tof_thread = threading.Thread(target=self._tof_thread)
        self.tof_thread.daemon = True
        self.tof_thread.start()
        
    def stop_tof_thread(self):
        """Stop the forward ToF thread. Internal method."""
        self.stop_event.set()
        try:
            if self.tof_thread and self.tof_thread.is_alive():
                self.tof_thread.join(timeout=2)
        except Exception as e:
            logging.error(f"Error in stopping ToF thread: {e}")

    def shutdown(self):
        """
        Properly shuts down the controller, threads and video stream
        NOTE: DOES NOT LAND THE DRONE
        """
        if self.is_running: # ensures shutdown is only run once, no matter the circumstances in which it is called
            logging.info("Shutdown initiated. Destroying all windows.")
            self.is_running = False
            self.stop_video_stream()
            self.stop_tof_thread()
            cv2.destroyAllWindows()
            for i in range(4):
                cv2.waitKey(1)
            self.drone.streamoff()
        else:
            logging.warning("Trying to shutdown, but already shut down previously.")

    def detect_markers(self, frame, display_frame, marker_size=19.0):
        """
        Detect ArUco markers and estimate pose.
        Returns values of the FIRST DETECTED VALID MARKER.

        Additionally, calculates the distance between the detected marker and a danger marker if present.

        :return:
            marker_detected (bool), marker_corners, marker_id (int), rotation_vec, translation_vec
        """
        # global CAMERA_MATRIX, DIST_COEFF
        aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters()
        corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)

        # Reset class attributes, for re-detection
        self.target_yaw = None
        self.exit_detected = False
        self.exit_distance_3D = None 

        self.invalid_ids = self.marker_client.get_invalid_markers(self.valid_ids)
        logging.debug(f"Invalid markers: {self.invalid_ids}")

        if ids is not None:
            detected_ids = ids.flatten()
            logging.debug(f"Detected IDs: {detected_ids}")

            # Check for exit markers - drone's subsequent behaviour is not coded here
            if set(detected_ids).intersection(self.exit_ids):
                self.exit_detected = True
                logging.info("Exit marker detected!")

            # Store first valid marker only
            self.valid_marker_info = {}   # only stores ONE valid marker info
            lockedon_marker_info = {} # either stores lockedon marker info or None
            danger_marker_info = {}  # stores ALL danger marker info

            for i, marker_id in enumerate(detected_ids):
                rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                    corners[i].reshape(1, 4, 2), marker_size, params.CAMERA_MATRIX, params.DIST_COEFF
                )
                x, y, z = tvecs[0][0]
                euclidean_distance = np.sqrt(x*x + y*y + z*z)

                if marker_id in self.valid_ids and (marker_id not in self.invalid_ids or marker_id == self.markernum_lockedon):
                    if not self.valid_marker_info:  # Ensures only first valid marker is stored
                        self.valid_marker_info = {
                            "id": int(marker_id),  # Ensure ID is a Python int
                            "position": (x, y, z),
                            "distance": euclidean_distance,
                            "corners": corners[i],
                            "rvecs": rvecs[0],
                            "tvecs": tvecs[0]
                        }

                    if marker_id == self.markernum_lockedon:    # Stores marker info of previously locked on ID
                        lockedon_marker_info = {
                            "id": int(marker_id),  # Ensure ID is a Python int
                            "position": (x, y, z),
                            "distance": euclidean_distance,
                            "corners": corners[i],
                            "rvecs": rvecs[0],
                            "tvecs": tvecs[0]
                        }
                elif marker_id in self.danger_ids:  # Store danger marker details
                    danger_marker_info[marker_id] = {
                        "position": (x, y, z),
                        "distance": euclidean_distance,
                        "corners": corners[i],
                        "rvecs": rvecs[0],
                        "tvecs": tvecs[0]
                    }
                # If it's an exit marker, compute its distance and yaw
                elif marker_id in self.exit_ids:
                    x, y, z = tvecs[0][0]
                    euclidean_distance = np.sqrt(x*x + y*y + z*z)
                    self.exit_distance_3D = euclidean_distance
                    # Convert rotation vector to rotation matrix
                    R, _ = cv2.Rodrigues(rvecs[0])
                    # Calculate yaw (rotation around Z-axis)
                    yaw = np.arctan2(R[1, 0], R[0, 0])  # Yaw in radians
                    self.target_yaw = np.degrees(yaw)
                    logging.debug(f"Exit marker yaw: {self.target_yaw:.2f}°")

            if lockedon_marker_info:    # ie. if lockedon_marker is still detected, even if its not the first detection, discard the valid_marker in favour of locked_on marker
                self.valid_marker_info = lockedon_marker_info

            # Resets only if no danger markers detected for 20 frames (ensures drone does NOT need to see both victim and danger in the last frame)
            if danger_marker_info == {}:    # i.e. no danger markers detected in last frame
                self.no_danger_count += 1
            else:
                self.no_danger_count = 0

            if self.no_danger_count >= 20:
                self.shortest_danger_distance = float("inf")  
                self.nearest_danger_id = None
                self.nearest_danger_data = None  
                self.danger_offset = (0,0,0)
                
            else:
                logging.debug("5 MAR DEBUG: Danger ID and data not resetting! Great!")

            # Compute distance of all detected danger markers to the single valid marker, then finds and save the ID and data of the nearest
            if self.valid_marker_info and not self.nearest_danger_id:       
                for danger_id, danger_data in danger_marker_info.items():
                    dx = self.valid_marker_info["position"][0] - danger_data["position"][0]
                    dy = self.valid_marker_info["position"][1] - danger_data["position"][1]
                    dz = self.valid_marker_info["position"][2] - danger_data["position"][2]
                    computed_distance = np.sqrt(dx*dx + dy*dy + dz*dz)

                    if computed_distance < self.shortest_danger_distance:
                        self.shortest_danger_distance = computed_distance
                        self.nearest_danger_id = danger_id
                        self.nearest_danger_data = danger_data
                        self.danger_offset = (int(dx), int(dy), int(dz))  

            # Return the first valid marker detected
            if self.valid_marker_info:
                self.set_distance(self.valid_marker_info["distance"])
                return True, self.valid_marker_info["corners"], self.valid_marker_info["id"], self.valid_marker_info["rvecs"], self.valid_marker_info["tvecs"]

        self.set_distance(None)
        return False, None, None, None, None

    def detect_danger(self, frame, marker_size=19.0) -> int:
        """
        Detect danger markers in the frame and return the ID and distance of the nearest one.
        
        :param frame: Camera frame to analyze
        :param marker_size: Size of the ArUco marker in cm
        :return: (nearest_danger_id, nearest_danger_distance) 
                If no danger markers detected, returns (None, 0)
        """
        aruco_dict = aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
        parameters = aruco.DetectorParameters()
        corners, ids, rejected = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
        
        # Initialize return values
        nearest_danger_id = None
        nearest_danger_distance = float("inf")
        
        if ids is not None:
            detected_ids = ids.flatten()
            
            for i, marker_id in enumerate(detected_ids):
                # Only process danger markers
                if marker_id in self.danger_ids:
                    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                        corners[i].reshape(1, 4, 2), marker_size, params.CAMERA_MATRIX, params.DIST_COEFF
                    )
                    
                    # Calculate distance
                    x, y, z = tvecs[0][0]
                    euclidean_distance = np.sqrt(x*x + y*y + z*z)
                    
                    # Update nearest danger marker if this one is closer
                    if euclidean_distance < nearest_danger_distance:
                        nearest_danger_distance = euclidean_distance
                        nearest_danger_id = int(marker_id)
                        
                        # Log detection
                        logging.debug(f"Danger marker detected: ID {nearest_danger_id}, distance {nearest_danger_distance:.2f}")
        
        # If no danger marker detected, return (None, 0)
        if nearest_danger_id is None:
            return None
        
        return int(nearest_danger_distance)