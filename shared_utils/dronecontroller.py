import threading
from threading import Lock
from customtello import CustomTello
import time
from djitellopy import Tello

class DroneController:
    def __init__(self, pi_id, tag_id):
        self.drone = initialize_drone()
        self.drone_id = pi_id
        self.drone_uwbtag = tag_id
        self.frame = None
        self.frame_lock = Lock()
        self.distance = [None]*25
        self.distance_lock = Lock()
        self.marker_center = None
        self.marker_pose = Lock()
        self.marker_x_lock = Lock()
        self.marker_x = [None]*25
        #self.startpose_lock = Lock()
        self.is_running = True
        self.has_taken_off = False
        self.movement_completed = False
        self.is_centered = False
        self.valid_ids = set(range(1, 4)) 
        self.invalid_ids = set(range(11, 14))
        self.bonus_victims = set(range(21, 24))
        self.target_marker_id = None
        self.total_marker = self.valid_ids | self.bonus_victims | self.invalid_ids

        # UWB Thread
        self.marker_position = None
        self.marker_position_lock = Lock()

        self.landing_position = None
        self.marker_list = []
        self.marker_list_lock = Lock()
        self.start_pose = []
        # self.heading = None
        self.sh_lock = Lock()
        self.marker_priority_list = set()
        self.marker_client = None
        self.heading_lock = Lock()
        # Waypoint control
        self.group_num = 1

        # Downward vision control
        self.using_downvision = False
        self.center_threshold = 20
        self.consecutive_centered_frames = 0
        self.required_centered_frames = 15
        self.rc_speed_scale = 0.3

        self.command_lock = Lock()
        self.last_distance_ts = 0.0

        self.rvec = None
        self.tvec = None
        self.rtlock = Lock()

    def get_marker_list(self):
        with self.marker_list_lock:
            return self.marker_list
        
    def set_marker_list(self, marker_list):
        with self.marker_list_lock:
            self.marker_list = marker_list
        
    def get_frame(self):
        with self.frame_lock:
            return self.frame.copy() if self.frame is not None else None
    
    def get_rtvec(self):
        with self.rtlock:
            return self.rvec, self.tvec
    
    def get_heading(self):
        with self.heading_lock:
            return self.drone.get_yaw()
        
    def set_rtvec(self, rVec, tVec):
        with self.rtlock:
            self.rvec = rVec
            self.tvec = tVec

    def set_frame(self, frame):
        with self.frame_lock:
            self.frame = frame

    def get_distance(self, id):
        with self.distance_lock:
            return self.distance[id]

    def set_distance(self, distance, id):
        with self.distance_lock:
            self.distance[id] = distance
            self.last_distance_ts = time.time()

    def get_marker_x(self, id): #Get the target marker
        with self.marker_x_lock:
            return self.marker_x[id]

    def set_marker_x(self, x, id):
        with self.marker_x_lock:
            self.marker_x[id] = x

    def set_marker_center(self, marker_x, marker_y):
        with self.marker_pose:
            self.marker_center = (marker_x, marker_y)
    
    def get_marker_center(self):
        with self.marker_pose:
            return self.marker_center
    
    def set_latest_uwb(self, uwb_position):
        with self.marker_position_lock:
            self.marker_position = uwb_position
    
    def get_latest_uwb(self):
        with self.marker_position_lock:
            return self.marker_position

def initialize_drone():
    drone = CustomTello()
    drone.connect()
    print(f"Battery Level: {drone.get_battery()}%")
    drone.streamon()
    try:
        drone.set_video_resolution(Tello.RESOLUTION_480P)
    except:
        pass
    time.sleep(1)
    return drone