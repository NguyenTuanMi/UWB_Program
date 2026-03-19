import cv2
from cv2 import aruco
import numpy as np
from djitellopy import Tello
import time
import threading
from threading import Lock
from UWB_Manipulation.UWB_Reader import get_target_position
from swarmserver.swarmserverclientnew_demo import MarkerClient
import math
import json

dis = []
pos = [0, 0]
waypoints = []
# ============================================================
# === Drone Controller Class
# ============================================================
pi_id = 5
host = f'192.168.0.{100+pi_id}'
control_port = 9000 + pi_id
state_port = 8000 + pi_id
video_port = 11100 + pi_id
tag_id = 6
current_detected_markers = []

group_1 = [5,6,9,10]
group_2 = []
group_3 = []
group_4 = []
course = 0
uwb_ground_height = 60
marker_list = []
fire_marker_list = []
counter = 0

class CustomTello(Tello):
    def __init__(self):
        
        global host, control_port, state_port, video_port

        
        # Store custom configuration
        self.TELLO_IP = host
        self.CONTROL_UDP_PORT = control_port
        self.STATE_UDP_PORT = state_port
        self.VS_UDP_PORT = video_port
        
        Tello.STATE_UDP_PORT = state_port
        Tello.CONTROL_UDP_PORT = control_port
        
        # Call parent's init with our custom host
        super().__init__(host, retry_count=30)
        
        # Override the connection parameters
        self.address = (self.TELLO_IP, self.CONTROL_UDP_PORT)
        
        # Override video port
        self.vs_udp_port = video_port

        self.latest_frame = None
        self.frame_lock = threading.Lock()

class DroneController:
    def __init__(self):
        self.drone = initialize_drone()
        self.drone_id = 5
        self.drone_uwbtag = 5
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
        self.marker_positions = {}
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

# ============================================================
# === Utility Functions
# ============================================================
def uwb_reading(drone):
    uwb_raw = (0,0,0)
    retry_count = 0
    while uwb_raw == (0,0,0) and retry_count < 10:
        uwb_raw = get_target_position(tag_id)
        retry_count += 1
    if uwb_raw == (0,0,0):
        return [0,0]
    uwb_pos = [uwb_raw[0]*100, uwb_raw[1]*100]    
    return uwb_pos

def uwb_correction(drone):
    global pos
    uwb_raw = (0,0,0)
    retry_count = 0
    while uwb_raw == (0,0,0) and retry_count < 15:
        uwb_raw = get_target_position(tag_id)
        retry_count += 1
    if uwb_raw == (0,0,0):
        return "No UWB data"
    uwb_pos = [uwb_raw[0]*100, uwb_raw[1]*100]
    uwb_height = uwb_raw[2]*100
    height = uwb_height - uwb_ground_height
    print(f"Height: {height}")

    x_diff = pos[0] - uwb_pos[0]
    y_diff = pos[1] - uwb_pos[1]
    heading = drone.get_yaw()
    x_corr = x_diff*math.cos(math.radians(heading)) - y_diff*math.sin(math.radians(heading))
    y_corr = y_diff*math.cos(math.radians(heading)) + x_diff*math.sin(math.radians(heading))
    print(f"Dead Reckoning Position: {pos}, UWB Position: {uwb_pos}")
    print(f"Deviation X: {x_corr}, Y: {y_corr}")
    if abs(int(x_corr)) >= 30 or abs(int(y_corr)) >= 30:   
        try: 
            drone.go_xyz_speed(int(y_corr), int(-x_corr),0,100)
            time.sleep(2)
        except:
            pass

def get_distance_with_retry(controller, id, max_attempts=50):
    for attempt in range(max_attempts):
        d = controller.get_distance(id)
        print(f"Current distance to marker {d}")
        if d is not None:
            return d
        time.sleep(0.1)
    return None

def get_marker_x_with_retry(controller, id, max_attempts=10):
    for attempt in range(max_attempts):
        d = controller.get_marker_x(id)
        if d is not None:
            return d
        time.sleep(0.1)
    return None

def emergency_land(drone):
    print("\nEmergency landing initiated!")
    try:
        drone.send_rc_control(0, 0, 0, 0)
        drone.emergency()
    except:
        pass
    time.sleep(0.5)

def initialize_drone():
    drone = CustomTello()
    drone.connect()
    print(f"Battery Level: {drone.get_battery()}%")

    try:
        drone.streamoff()
        time.sleep(0.8)
    except:
        pass
    drone.streamon()
    try:
        drone.set_video_resolution(Tello.RESOLUTION_480P)
    except:
        pass
    time.sleep(1)
    return drone

def get_calibration_parameters(controller):
    K = np.array([
        [473.11891765, 0.000000, 323.13378419],
        [0.000000, 475.40414375, 234.45123335],
        [0.000000, 0.000000, 1.000000]
    ])
    D = np.array([0.04172334, 0.38719427, -0.00921904, 0.00348033, -0.70063595])

    if controller.using_downvision:
        K = np.array([
            [232.08608036, 0.000000, 152.2358733], 
            [0.000000, 232.64995134, 124.98737218], 
            [0.000000, 0.000000, 1.000000]
        ])
    
        D = np.array([5.26408126e-01, -1.52051597e+00, 1.20937906e-02, -1.34771857e-03, 1.29077550e+00])
    return K, D

def brake(drone, ms=300):
    try:
        drone.send_rc_control(0, 0, 0, 0)
    finally:
        time.sleep(ms/1000.0)

# ============================================================
# === ArUco Detection and Visualization
# ============================================================

def detect_marker_pose(gray_frame, controller,
                       aruco_dict_type=cv2.aruco.DICT_5X5_250,
                       marker_size_m=0.19):
    K, D = get_calibration_parameters(controller)
    adict = aruco.getPredefinedDictionary(aruco_dict_type)
    params = aruco.DetectorParameters()

    try:
        corners, ids, _ = aruco.detectMarkers(gray_frame, adict, parameters=params)
    except cv2.error:
        return None, None, None, None

    if ids is None or len(corners) == 0:
        return None, None, None, None

    # valid_corners, valid_ids = [], []
    # for i, mid in enumerate(ids):
    #     m = int(mid[0])
    #     if m in controller.valid_ids and m not in controller.invalid_ids:
    #         c = np.asarray(corners[i], dtype=np.float32)
    #         valid_corners.append(c)
    #         valid_ids.append(mid)

    # if not valid_corners:
    #     return None, None, None, None

    try:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size_m, K, D)
    except cv2.error:
        return corners, ids, None, None
    #ontroller.set_rtvec(rvecs, tvecs)
    return corners, ids, rvecs, tvecs

def draw_pose_axes(frame, corners, ids, rvecs, tvecs, controller, is_fire):
    if ids is None or rvecs is None or tvecs is None:
        return frame
    K, D = get_calibration_parameters(controller)
    for i in range(len(ids)):
        try:
            cv2.drawFrameAxes(frame, K, D, rvecs[i], tvecs[i], 0.05)
        except cv2.error:
            continue

        mc = np.mean(corners[i][0], axis=0).astype(int)
        cv2.circle(frame, (mc[0], mc[1]), 5, (0, 0, 255), -1)

        x_cm = float(tvecs[i][0][0] * 100.0)
        y_cm = float(tvecs[i][0][1] * 100.0)
        z_cm = float(tvecs[i][0][2] * 100.0)
        horiz_cm = (x_cm**2 + z_cm**2) ** 0.5
        
        controller.set_rtvec(x_cm, y_cm)
        color = (0,255,0)
        if is_fire:
            color = (255, 0, 0)
        cv2.putText(frame, f"X:{x_cm:+.1f} Y:{y_cm:+.1f} Z(fwd):{z_cm:.1f} H:{horiz_cm:.1f}",
                    (10, 30 + 22*i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return frame

# ============================================================
# === Video Thread
# ============================================================

def video_thread(controller):
    global current_detected_markers
    print("Starting video thread...")
    frame_reader = controller.drone.get_frame_read()
    if frame_reader is None:
        print("Failed to get frame reader!")
        return

    alpha = 0.3
    is_fire = False
    while controller.is_running:
        frame = frame_reader.frame
        if frame is None or not hasattr(frame, "shape") or frame.size == 0:
            time.sleep(0.02)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rvecs, tvecs = detect_marker_pose(gray, controller)

        if ids is None:
            current_detected_markers = []
        else: 
            current_detected_markers = [id[0] for id in ids]
        if ids is not None and rvecs is not None and tvecs is not None:
            for i in range(len(ids)):
                if ids[i][0] == 0 or ids[i][0] > 25:
                    continue
                if ids[i][0] not in marker_list:
                    marker_list.append(ids[i][0])
                    print(f"New marker {ids[i][0]} is detected")

                if ids[i][0] in controller.valid_ids or ids[i][0] in controller.bonus_victims:
                    is_fire = False
                elif ids[i][0] in controller.invalid_ids:
                    is_fire = True
                    if ids[i][0] not in fire_marker_list:
                        fire_marker_list.append(ids[i][0])
                        print(f"New fire marker {ids[i][0]} is detected")
                
                x_cm = float(tvecs[i][0][0] * 100.0)
                z_cm = float(tvecs[i][0][2] * 100.0)

                # Compute horizontal (XZ-plane) distance
                horiz_cm = (x_cm**2 + z_cm**2) ** 0.5
            
                prev = controller.get_distance(ids[i][0])
                if prev is None:
                    controller.set_distance(horiz_cm, ids[i][0])
                else:
                    controller.set_distance((1 - alpha) * prev + alpha * horiz_cm, ids[i][0])
                    # mc = np.mean(corners[0][0], axis=0)
                    
                controller.set_marker_x(x_cm, ids[i][0])
                frame = draw_pose_axes(frame, corners, ids, rvecs, tvecs, controller, is_fire)
            #print(f"Detected marker ID with opencv: {ids}")

                # corner = np.array(corners[i], copy=True)
                # corner.reshape((4,2))
                # (top_left, top_right, bottom_right, bottom_left) = corner[0]
                marker_center_x = int(tvecs[i][0][0] * 100.0)
                marker_center_y = int(tvecs[i][0][1] * 100.0) 
                controller.set_marker_center(marker_center_x, marker_center_y)    
                # else:
                #     controller.set_distance(None, ids[])
                #     controller.set_marker_x(None)

        controller.set_frame(frame)

# ============================================================
# === Centering & Downward Landing
# ============================================================

def center_on_marker(controller, id, marker_client):
    marker_client.send_update('marker', marker_id=int(id), detected=True)
    controller.set_marker_x(None, id)
    time.sleep(1)
    marker_x = get_marker_x_with_retry(controller, id) 
    frm = controller.get_frame()
    if marker_x is None or frm is None:
        print("Searching for marker...")
        controller.drone.send_rc_control(0, 0, 0, 0)
        return False

    x_error = marker_x
    threshold = 10
    # print("Position of the current marker")

    if abs(x_error) > threshold:
        yaw_speed = int(x_error/5)
        controller.drone.send_rc_control(0, 0, 0, yaw_speed)
        controller.is_centered= False
        marker_client.send_update('marker', marker_id=int(id), detected=True)
        return False
    else:
        if not controller.is_centered:
            print("Marker centered!")
            controller.drone.send_rc_control(0, 0, 0, 0)
            time.sleep(0.3)
            controller.is_centered = True
        return True

def downward_center_and_land(controller, target_marker_id, marker_client):
    marker_client.send_update('marker', marker_id=int(target_marker_id), detected=True)
    marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True)
    for marker_id in marker_list:
        if marker_id != target_marker_id:
            marker_client.send_update('marker', marker_id=int(marker_id), detected=False)
    print("\nSwitching to downward camera for smooth landing...")
    for i in range(3):
        controller.drone.send_command_with_return("downvision 1")
        time.sleep(0.5)
    controller.using_downvision = True
    time.sleep(2)

    # --- basic parameters ---         # cm/s base RC speed
    center_tolerance = 8      # pixel tolerance for "centered"
    descend_speed = -20        # cm/s downward
    min_height = 60            # stop when below this      # largest dx, dy we expect
    kx = 0.6
    ky = 0.6
    def rc_pulse(lr=0, fb=0, ud=0, dur=0.3):
        controller.drone.send_rc_control(int(lr), int(fb), int(ud), 0)
        time.sleep(float(dur))
        controller.drone.send_rc_control(0, 0, 0, 0)

    marker_client.send_update('marker', marker_id=int(target_marker_id), detected=True)
    marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True)
    while controller.is_running:
        frame = controller.get_frame() #Probably because the frame of the controller is still from the previous front camera
        marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True)
        if frame is None:
            time.sleep(0.1)
            continue
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rvecs, tvecs = detect_marker_pose(gray_frame, controller)
        
        marker_center = controller.get_marker_center()
        marker_client.send_update('marker', marker_id=int(target_marker_id), detected=True)
        marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True)
        for marker_id in marker_list:
            if marker_id != target_marker_id:
                marker_client.send_update('marker', marker_id=int(marker_id), detected=False)
        #   logging.info(f"Downward centering loop: marker_center={marker_center} at {time.time()}")
        print(f"INFO: Downward centering loop: marker_center={marker_center} at {time.time()}")
        if rvecs is not None and tvecs is not None:

            x_offset, y_offset = tvecs[0][0][0]*100, tvecs[0][0][1]*100

            if abs(x_offset) < center_tolerance and abs(y_offset) < center_tolerance:
                print("Landing.")
                break
            
            else:
                delta_x = x_offset*kx
                delta_y = y_offset*ky

                lr = int(-delta_y)
                fb = int(-delta_x)

                print(f"Offset big (offset x in cm = {x_offset}, offset_y in cm = {y_offset} → "
                  f"lr={lr:.0f}, fb={fb:.0f}")
                    
                controller.drone.send_rc_control(lr, fb, 0, 0)
                time.sleep(0.2)
                controller.drone.send_rc_control(0,0,0,0)
                continue
        else: 
            break
    print("Centered — descending slightly...")
    rc_pulse(0, 0, descend_speed)
    time.sleep(0.3)

    h = controller.drone.get_height()
    if h is not None and h <= min_height:
        print(f"Near ground (h={h} cm) — landing.")
    marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True)
    controller.drone.send_rc_control(0, 0, 0, 0)
    controller.drone.land()
    controller.movement_completed = True
    print("Landed safely.")

# ============================================================
# === Movement Thread
# ============================================================

def scan_marker(controller, marker_client, release = False):
    # global marker_client
    
    for id in marker_list:
        print(f"ID {id} is being scanned")
        if marker_client.is_marker_available(id) and id in controller.valid_ids:
            marker_client.send_update('marker', marker_id=int(id), detected=True)
            controller.set_distance(None, id)
            time.sleep(0.5)
            dist = get_distance_with_retry(controller, id, max_attempts=5)
            if dist is None:
                continue
            else: 
                locate_marker(controller, id, marker_client=marker_client)
            print("Has reached here")


def scan_for_marker(controller, marker_client):
    enter_searching_zone = False
    while controller.get_heading() > 5 or controller.get_heading() < -5 or not enter_searching_zone:
        if controller.get_heading() > 5:
            enter_searching_zone = True
        for id in list(marker_list):
            controller.set_distance(None, id)
            time.sleep(0.5)
            dist = get_distance_with_retry(controller, id, max_attempts=5)
            if dist is None:
                continue
            if marker_client.is_marker_available(id) and id in controller.valid_ids:
                marker_client.send_update('marker', marker_id=int(id), detected=True)
                controller.drone.send_rc_control(0, 0, 0, 0)
                print(f"Measuring Marker {id}'s position...")
                status = f"Measuring Marker {id}'s position..."
                locate_marker(controller,id, marker_client)
        controller.drone.send_rc_control(0, 0, 0, 10)
        time.sleep(1)
        controller.drone.send_rc_control(0, 0, 0, 0)
        time.sleep(1)
    
    # This part is to align the drone back to the original course heading
    heading = controller.get_heading()
    print(f"Current heading: {heading}")
    print(f"Current course: {course}")
    if abs(heading-course) <= 180:
        controller.drone.rotate_counter_clockwise(heading-course)
    elif heading-course < -180:
        controller.drone.rotate_counter_clockwise(360 + heading-course)
    else:
        controller.drone.rotate_counter_clockwise(heading-course - 360)

def scan_for_fire(controller, marker_client):
    enter_searching_zone = True
    while controller.get_heading() > 5 or controller.get_heading() < -5 or not enter_searching_zone:
        if controller.get_heading() > 5:
            enter_searching_zone = True
        for id in list(fire_marker_list):
            controller.set_distance(None, id)
            time.sleep(0.5)
            dist = get_distance_with_retry(controller, id, max_attempts=5)
            if dist is None:
                continue
            if marker_client.is_marker_available(id) and id in controller.invalid_ids:
                marker_client.send_update('marker', marker_id=int(id), detected=True)
                controller.drone.send_rc_control(0, 0, 0, 0)
                print(f"Measuring Marker {id}'s position...")
                status = f"Measuring Marker {id}'s position..."
                locate_fire_marker(controller,id, marker_client)
            # locate_marker(controller, id)
        controller.drone.send_rc_control(0, 0, 0, 10)
        time.sleep(1)
        controller.drone.send_rc_control(0, 0, 0, 0)
        time.sleep(1)

def locate_fire_marker(controller, id, marker_client):
    marker_client.send_update('marker', marker_id=int(id), detected=True)
    # while controller.is_running and controller.get_distance() is None:
    #     time.sleep(0.1)
    print("The second time has reached here")
    print("\nStep 1: Centering on marker...")
    while not center_on_marker(controller, id, marker_client) and controller.is_running:
        marker_client.send_update('marker', marker_id=int(id), detected=True)
        time.sleep(0.1)
    print("The third time has reached here")
    controller.set_distance(None, id)
    time.sleep(1)
    initial_distance = get_distance_with_retry(controller, id)
            
    if initial_distance is None:
        print(f"Marker {id} was lost")
        status = f"Marker {id} was lost"
        if id in marker_list:
            marker_list.pop(marker_list.index(id))           
            print("Could not detect marker for initial distance!")
        return

    print(f"Initial horizontal distance to marker: {initial_distance:.1f} cm")

    print("\nStep 2: Gradual horizontal approach...")
    total_forward_distance = initial_distance*0.9
    num_segments = 4
    initial_segment = int(total_forward_distance / num_segments)
    forward_per_segment = initial_segment

    try:
        current_altitude = controller.drone.get_height()  # in cm
    except Exception:
        current_altitude = 100  # fallback default if API fails
    print(f"Current altitude: {current_altitude} cm")
    target_altitude = 60

    total_descent = max(current_altitude - target_altitude, 0)
    descent_per_segment = int(total_descent / num_segments)

    for i in range(num_segments):
        marker_client.send_update('marker', marker_id=int(id), detected=True)
        if not controller.is_running:
            break
        if i != 0:
            controller.set_distance(None, id)
            time.sleep(0.5)
            dist = get_distance_with_retry(controller, id)
            if dist is None: 
                forward_per_segment = initial_segment
            else: 
                print("Recentering on marker (if visible)...")
                marker_client.send_update('marker', marker_id=int(id), detected=True)
                while not center_on_marker(controller, id, marker_client) and controller.is_running:
                    time.sleep(0.1)
                dist = get_distance_with_retry(controller, id, max_attempts=5)
                forward_per_segment = int(dist*0.9/(num_segments-(i)))
        print(f"\nExecuting segment {i+1}/{num_segments}")
        controller.drone.move_forward(forward_per_segment)
        time.sleep(1)
        brake(controller.drone, 300)
        time.sleep(0.6)
        
        descent_velocity = descent_per_segment/0.5
        controller.drone.send_rc_control(0, 0, -int(descent_velocity), 0)  # gentle down velocity (cm/s)
        time.sleep(0.5)
        controller.drone.send_rc_control(0, 0, 0, 0) 

        brake(controller.drone, 300)
        time.sleep(0.5)

        # print("Recentering on marker (if visible)...")
        # while not center_on_marker(controller, id) and controller.is_running:
        #     time.sleep(0.1)
        # max_center_attempts = 15  # about 1.5 seconds max

        # for _ in range(max_center_attempts):
        #     if not controller.is_running:
        #         break

        #     time.sleep(0.1)

        time.sleep(0.4)
    downward_center_and_land(controller, id, marker_client)
    
def locate_marker(controller, id, marker_client):
    marker_client.send_update('marker', marker_id=int(id), detected=True)
    # while controller.is_running and controller.get_distance() is None:
    #     time.sleep(0.1)
    print("The second time has reached here")
    print("\nStep 1: Centering on marker...")
    while not center_on_marker(controller, id, marker_client) and controller.is_running:
        marker_client.send_update('marker', marker_id=int(id), detected=True)
        time.sleep(0.1)
    print("The third time has reached here")
    controller.set_distance(None, id)
    time.sleep(1)
    initial_distance = get_distance_with_retry(controller, id)
            
    if initial_distance is None:
        print(f"Marker {id} was lost")
        status = f"Marker {id} was lost"
        if id in marker_list:
            marker_list.pop(marker_list.index(id))           
            print("Could not detect marker for initial distance!")
        return

    print(f"Initial horizontal distance to marker: {initial_distance:.1f} cm")

    print("\nStep 2: Gradual horizontal approach...")
    total_forward_distance = initial_distance*0.9
    num_segments = 4
    initial_segment = int(total_forward_distance / num_segments)
    forward_per_segment = initial_segment

    try:
        current_altitude = controller.drone.get_height()  # in cm
    except Exception:
        current_altitude = 100  # fallback default if API fails
    print(f"Current altitude: {current_altitude} cm")
    target_altitude = 60

    total_descent = max(current_altitude - target_altitude, 0)
    descent_per_segment = int(total_descent / num_segments)

    for i in range(num_segments):
        marker_client.send_update('marker', marker_id=int(id), detected=True)
        if not controller.is_running:
            break
        if i != 0:
            controller.set_distance(None, id)
            time.sleep(0.5)
            dist = get_distance_with_retry(controller, id)
            if dist is None: 
                forward_per_segment = initial_segment
            else: 
                print("Recentering on marker (if visible)...")
                marker_client.send_update('marker', marker_id=int(id), detected=True)
                while not center_on_marker(controller, id, marker_client) and controller.is_running:
                    time.sleep(0.1)
                dist = get_distance_with_retry(controller, id, max_attempts=5)
                forward_per_segment = int(dist*0.9/(num_segments-(i)))
        print(f"\nExecuting segment {i+1}/{num_segments}")
        controller.drone.move_forward(forward_per_segment)
        time.sleep(1)
        brake(controller.drone, 300)
        time.sleep(0.6)
        
        descent_velocity = descent_per_segment/0.5
        controller.drone.send_rc_control(0, 0, -int(descent_velocity), 0)  # gentle down velocity (cm/s)
        time.sleep(0.5)
        controller.drone.send_rc_control(0, 0, 0, 0) 

        brake(controller.drone, 300)
        time.sleep(0.5)

        # print("Recentering on marker (if visible)...")
        # while not center_on_marker(controller, id) and controller.is_running:
        #     time.sleep(0.1)
        # max_center_attempts = 15  # about 1.5 seconds max

        # for _ in range(max_center_attempts):
        #     if not controller.is_running:
        #         break

        #     time.sleep(0.1)

        time.sleep(0.4)
    scan_for_fire(controller, marker_client)
    downward_center_and_land(controller, id, marker_client)

def movement_thread(controller, marker_client):
    global pos
    print("Starting movement thread...")

    uwb_raw = (0,0,0)
    while uwb_raw == (0,0,0):
        uwb_raw = get_target_position(tag_id)
    uwb_pos = [uwb_raw[0]*100, uwb_raw[1]*100]
    uwb_ground_height = uwb_raw[2]*100
    print(f" Ground pos: {uwb_pos}")
    pos = uwb_pos
    
    controller.start_pose = uwb_pos

    marker_client.client_takeoff_simul([99], f'Battery: {controller.drone.get_battery()} Pos: {int(pos[0]), int(pos[1])}')
    print("Taking off...")
    controller.drone.takeoff()
    controller.has_taken_off = True
    controller.drone.move_up(30)
    time.sleep(1)
    
    execute_waypoints(controller, marker_client=marker_client)

def update_position(waypoints, position, orientation, distance=0):
    global pos

    rad = math.radians(orientation)
    pos[1] -= int(distance * math.cos(rad))
    pos[0] += int(distance * math.sin(rad))
    position["x"] += int(distance * math.cos(rad))
    position["y"] += int(distance * math.sin(rad))
    waypoints.append({"x": position["x"], "y": position["y"], "orientation": orientation, "distance": distance})

def validate_waypoints():

    global start_wpt

    if pi_id in group_1:
        with open('waypoint6.json', 'r') as f:
            data = json.load(f)
    '''
    elif drone_id in group_2:
        with open('waypoint_grp2.json', 'r') as f:
            data = json.load(f)
    elif drone_id in group_3:
        with open('waypoint_grp3.json', 'r') as f:
            data = json.load(f)
    elif drone_id in group_4:
        with open('waypoint_grp4.json', 'r') as f:
            data = json.load(f)
    else:
        with open('waypoints_uwb.json', 'r') as f:
            data = json.load(f)
    '''

    start_wpt = [data['wp'][0]['position_cm']['x'], data['wp'][0]['position_cm']['y']]
    print(f"Starting waypoint is {start_wpt}")
    
    valid = True
    for i, wp in enumerate(data['wp']): # wp is the datapoint for each waypoint (its coordinate, and distance/angle to next waypoint)
        # Check distance
        if wp['dist_cm'] < 20 and not i+1 == len(data['wp']):
            print(i, len(data['wp']))
            print(f"[WARNING] Waypoint {i+1} distance ({wp['dist_cm']}cm) is below minimum 20cm")
            valid = False
        if wp['dist_cm'] > 500:
            print(f"[INFO] Waypoint {i+1} distance ({wp['dist_cm']}cm) will be split into multiple commands")
        
        # Check angle
        if abs(wp['angle_deg']) > 360:
            print(f"[WARNING] Waypoint {i+1} angle ({wp['angle_deg']}°) exceeds 360 degrees")
            valid = False
    
        # Check for dist_cm = 0 condition (only valid if it is last waypoint; invalid otherwise.)
        if wp['dist_cm'] == 0 and i+1 == len(data['wp']):
            print(f"[INFO] Final Waypoint {i+1} distance is zero to indicate end of flight routine.")
        elif not wp['dist_cm'] == 0 and i+1 == len(data['wp']):
            print(f"[INFO] Final Waypoint {i+1} distance is NOT zero to indicate end of flight routine.") # TBC 7 Jan extra?
        elif wp['dist_cm'] == 0:
            print(f"[WARNING] Waypoint {i+1} distance is zero but is not the final waypoint. Invalid flight path.") # TBC 7 Jan extra?
            valid = False

    return valid

def execute_waypoints(controller, marker_client):
    global LAND_ID, FLYING_STATE, waypoints, status, course, pos, waypoint_id
    sx, sy = controller.start_pose
    waypoint_id = 0 # Waypoint 0 is the starting waypoint
    while marker_client.is_waypoint_available(waypoint_id) is False:
        print(f"waiting for waypoint {waypoint_id} to be available")
        controller.drone.send_rc_control(0, 0, 0, 0)
        time.sleep(1)
    marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
    print(int(start_wpt[1]-sy),int(sx-start_wpt[0]))
    if abs(int(start_wpt[1]-sy)) > 20 or  abs(int(sx-start_wpt[0])) > 20:
        if abs(int(start_wpt[1]-sy)) > 1500 or  abs(int(sx-start_wpt[0])) > 1500:
            print("More than 1500cm")
            controller.drone.go_xyz_speed(int((start_wpt[1]-sy)/4), int((sx-start_wpt[0])/4), 0, 100)
            pos[0] += (start_wpt[0] - sx)/4
            pos[1] += (start_wpt[1] - sy)/4
            uwb_correction(controller.drone)
            controller.drone.go_xyz_speed(int((start_wpt[1]-sy)/4), int((sx-start_wpt[0])/4), 0, 100)
            pos[0] += (start_wpt[0] - sx)/4
            pos[1] += (start_wpt[1] - sy)/4
            uwb_correction(controller.drone)
            controller.drone.go_xyz_speed(int((start_wpt[1]-sy)/4), int((sx-start_wpt[0])/4), 0, 100)
            pos[0] += (start_wpt[0] - sx)/4
            pos[1] += (start_wpt[1] - sy)/4
            uwb_correction(controller.drone)
            controller.drone.go_xyz_speed(int((start_wpt[1]-sy)/4), int((sx-start_wpt[0])/4), 0, 100)
            pos[0] += (start_wpt[0] - sx)/4
            pos[1] += (start_wpt[1] - sy)/4
        elif abs(int(start_wpt[1]-sy)) > 1000 or  abs(int(sx-start_wpt[0])) > 1000:
            print("More than 1000cm")
            controller.drone.go_xyz_speed(int((start_wpt[1]-sy)/3), int((sx-start_wpt[0])/3), 0, 100)
            pos[0] += (start_wpt[0] - sx)/3
            pos[1] += (start_wpt[1] - sy)/3
            uwb_correction(controller.drone)
            controller.drone.go_xyz_speed(int((start_wpt[1]-sy)/3), int((sx-start_wpt[0])/3), 0, 100)
            pos[0] += (start_wpt[0] - sx)/3
            pos[1] += (start_wpt[1] - sy)/3
            uwb_correction(controller.drone)
            controller.drone.go_xyz_speed(int((start_wpt[1]-sy)/3), int((sx-start_wpt[0])/3), 0, 100)
            pos[0] += (start_wpt[0] - sx)/3
            pos[1] += (start_wpt[1] - sy)/3
        elif abs(int(start_wpt[1]-sy)) > 500 or  abs(int(sx-start_wpt[0])) > 500:
            print("More than 500cm")
            controller.drone.go_xyz_speed(int((start_wpt[1]-sy)/2), int((sx-start_wpt[0])/2), 0, 100)
            pos[0] += (start_wpt[0] - sx)/2
            pos[1] += (start_wpt[1] - sy)/2
            uwb_correction(controller.drone)
            controller.drone.go_xyz_speed(int((start_wpt[1]-sy)/2), int((sx-start_wpt[0])/2), 0, 100)
            pos[0] += (start_wpt[0] - sx)/2
            pos[1] += (start_wpt[1] - sy)/2
        elif abs(int(start_wpt[1]-sy)) <= 500 and  abs(int(sx-start_wpt[0])) <= 500:
            print("Less than 500cm")
            controller.drone.go_xyz_speed(int(start_wpt[1]-sy), int(sx-start_wpt[0]), 0, 100)
            pos[0] += start_wpt[0] - sx
            pos[1] += start_wpt[1] - sy
        print(f"Starting position: {pos}")
    else:
        print("Already at starting waypoint")
    # marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
    time.sleep(3)
    uwb_correction(controller.drone)
    # marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
    heading = controller.get_heading()
    if abs(heading) > 1:
        controller.drone.rotate_counter_clockwise(int(heading))

    try:

        # Initialize position and orientation
        abs_position = {"x": start_wpt[0], "y": start_wpt[1]}  # in cm
        orientation = 180  # Starting heading in degrees (assuming 180 as the initial heading)
        
        if pi_id in group_1:
            with open('waypoint6.json', 'r') as f:
                data = json.load(f)
        elif pi_id in group_2:
            with open('waypoint_grp2.json', 'r') as f:
                data = json.load(f)
        elif pi_id in group_3:
            with open('waypoint_grp3.json', 'r') as f:
                data = json.load(f)
        elif pi_id in group_4:
            with open('waypoint_grp4.json', 'r') as f:
                data = json.load(f)
        else:
            with open('waypoints_uwb.json', 'r') as f:
                data = json.load(f) 
        
        # marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
        # Execute each waypoint
        for wp in data['wp']:
            # Handle rotation
            status = "Orienting"
            if wp['angle_deg'] != 0:
                orientation += wp['angle_deg']
                orientation %= 360  # Keep orientation within 0 to 360 degrees
                if orientation > 180:
                    orientation -= 360  # Convert to -180 to 180 range
                
                course = int(-orientation) + 180
                if course > 180:
                    course -= 360 # Convert to -180 to 180 range

                # marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
                time.sleep(2)
                
                while abs(course - controller.get_heading()) > 3:
                    heading = controller.get_heading()
                    if -180 < course - heading < 180:
                        controller.drone.send_command_without_return(f"cw {int(course - heading)}")
                    elif course - heading > 180:
                        controller.drone.send_command_without_return(f"cw {int(course - heading) - 360}")
                    else:
                        controller.drone.send_command_without_return(f"cw {int(course - heading) + 360}")
                    time.sleep(4)
                
                scan_marker(controller, marker_client=marker_client) #Check the availability of the marker_client

                # marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
                update_position(waypoints, abs_position, orientation)
                uwb_correction(controller.drone)

            #heading = controller.get_heading()
            # Check if drone is facing course direction before flying forward
            while abs(course - controller.get_heading()) > 5:
                heading = controller.get_heading()
                if -180 < course - heading < 180:
                    controller.drone.send_command_without_return(f"cw {int(course - heading)}")
                elif course - heading > 180:
                    controller.drone.send_command_without_return(f"cw {int(course - heading) - 360}")
                else:
                    controller.drone.send_command_without_return(f"cw {int(course - heading) + 360}")
                time.sleep(4)

            # Handle forward movement in 200 cm increments
            distance = wp['dist_cm']

            status = "Proceeding forward"
            if distance > 250:
                while distance > 200:
                    waypoint_id += 1
                    print(f"Drone at proceeding to {waypoint_id}")
                    while marker_client.is_waypoint_available(waypoint_id) is False:
                        print(f"waiting for waypoint {waypoint_id} to be available")
                        controller.drone.send_rc_control(0, 0, 0, 0)
                        # marker_client.send_update('waypoint', marker_id=waypoint_id-1, detected=True)
                        time.sleep(1)
                    marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
                    try:
                        controller.drone.move_forward(200)
                    except:
                        waypoint_id -= 1
                        continue
                    
                    print(f"Drone at to Waypoint {waypoint_id}")
                    scan_marker(controller, marker_client, True) 
                    update_position(waypoints, abs_position, orientation, 200)
                    time.sleep(2)
                    uwb_correction(controller.drone)
                    time.sleep(2)
                    # marker_client.send_update('waypoint', marker_id=waypoint_id-1, detected=False)
                    scan_for_marker(controller, marker_client)
                    time.sleep(1)

                    distance -= 200
                    time.sleep(0.5)
                
            # Move remaining distance (if between 20 and 50 cm)
            if distance > 20:
                waypoint_id += 1
                print(f"Drone at proceeding to {waypoint_id}")
                '''
                while marker_client.is_waypoint_available(waypoint_id) is False:
                    print(f"waiting for waypoint {waypoint_id} to be available")
                    drone.send_rc_control(0, 0, 0, 0)
                    marker_client.send_update('waypoint', marker_id=waypoint_id-1, detected=True)
                    time.sleep(1)
                marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
                '''
                try:
                    controller.drone.move_forward(distance)
                except:
                    waypoint_id -= 1
                    continue
                
                print(f"Drone at to Waypoint {waypoint_id}")
                scan_marker(controller, marker_client, True)
                update_position(waypoints, abs_position, orientation, distance)
                time.sleep(3)
                uwb_correction(controller.drone)
                marker_client.send_update('waypoint', marker_id=waypoint_id-1, detected=False)
                scan_for_marker(controller, marker_client)
                time.sleep(1)
    
    except Exception as e:
        print(f"Error occurred: {e}")
    
    finally:
        if controller.drone.is_flying:
            print("Landing...")
            status = "Landing..."
            controller.drone.land()
            controller.drone.streamoff()
        print("Mission completed!")

# ============================================================
# === Display Loop
# ============================================================

def display_loop(controller):
    cv2.namedWindow(f"Tello {pi_id} Camera", cv2.WINDOW_NORMAL)
    while controller.is_running:
        frame = controller.get_frame()
        if frame is not None and frame.size:
            cv2.imshow(f"Tello {pi_id} Camera", frame)
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q')):
            controller.is_running = False
            break
    cv2.destroyAllWindows()


# ============================================================
# === Main
# ============================================================

def main():
    controller = DroneController()
    markerclient = MarkerClient(drone_id=controller.drone_id)
    # controller.marker_client = markerclient
    try:
        video_handler = threading.Thread(target=video_thread, args=(controller,), daemon=True)
        move_handler  = threading.Thread(target=movement_thread, args=(controller,markerclient,), daemon=True)

        video_handler.start()
        time.sleep(1.0)
        move_handler.start()
        display_loop(controller)

        controller.is_running = False
        video_handler.join(timeout=2)
        move_handler.join(timeout=2)
    finally:
        try: controller.drone.send_rc_control(0,0,0,0)
        except: pass
        try:
            if controller.has_taken_off and not controller.movement_completed:
                controller.drone.land()
        except: pass
        try: controller.drone.streamoff()
        except: pass
        cv2.destroyAllWindows()


if __name__ == "__main__":
    print("Validating waypoints...")
    if validate_waypoints():
        print("Validation passed. Starting execution...")
        main()
    else:
        print("Validation failed. Please check warnings above.")
