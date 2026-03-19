import cv2
from cv2 import aruco
import numpy as np
from djitellopy import Tello
import time
import threading
from threading import Lock

# ============================================================
# === Drone Controller Class
# ============================================================
pi_id = 5
host = f'192.168.0.{100+pi_id}'
control_port = 9000 + pi_id
state_port = 8000 + pi_id
video_port = 11100 + pi_id
tag_id = 5

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
        super().__init__(host)
        
        # Override the connection parameters
        self.address = (self.TELLO_IP, self.CONTROL_UDP_PORT)
        
        # Override video port
        self.vs_udp_port = video_port

        self.latest_frame = None
        self.frame_lock = threading.Lock()

class DroneController:
    def __init__(self):
        self.drone = initialize_drone()
        self.frame = None
        self.frame_lock = Lock()
        self.distance = None
        self.distance_lock = Lock()
        self.marker_center = None
        self.marker_pose = Lock()
        self.marker_x_lock = Lock()
        self.is_running = True
        self.has_taken_off = False
        self.movement_completed = False
        self.is_centered = False
        self.valid_ids = set(range(1, 9))
        self.invalid_ids = set(range(11, 15))
        self.target_marker_id = None
        self.marker_positions = {}
        self.landing_position = None

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

    def get_frame(self):
        with self.frame_lock:
            return self.frame.copy() if self.frame is not None else None
    
    def get_rtvec(self):
        with self.rtlock:
            return self.rvec, self.tvec
    
    def set_rtvec(self, rVec, tVec):
        with self.rtlock:
            self.rvec = rVec
            self.tvec = tVec

    def set_frame(self, frame):
        with self.frame_lock:
            self.frame = frame

    def get_distance(self):
        with self.distance_lock:
            return self.distance

    def set_distance(self, distance):
        with self.distance_lock:
            self.distance = distance
            self.last_distance_ts = time.time()

    def get_marker_x(self):
        with self.marker_x_lock:
            return self.marker_x

    def set_marker_x(self, x):
        with self.marker_x_lock:
            self.marker_x = x

    def set_marker_center(self, marker_x, marker_y):
        with self.marker_pose:
            self.marker_center = (marker_x, marker_y)
    
    def get_marker_center(self):
        with self.marker_pose:
            return self.marker_center

# ============================================================
# === Utility Functions
# ============================================================

def get_distance_with_retry(controller, max_attempts=30):
    for attempt in range(max_attempts):
        d = controller.get_distance()
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

    valid_corners, valid_ids = [], []
    for i, mid in enumerate(ids):
        m = int(mid[0])
        if m in controller.valid_ids and m not in controller.invalid_ids:
            c = np.asarray(corners[i], dtype=np.float32)
            valid_corners.append(c)
            valid_ids.append(mid)

    if not valid_corners:
        return None, None, None, None

    try:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(valid_corners, marker_size_m, K, D)
    except cv2.error:
        return valid_corners, np.array(valid_ids), None, None
    #ontroller.set_rtvec(rvecs, tvecs)
    return valid_corners, np.array(valid_ids), rvecs, tvecs

def draw_pose_axes(frame, corners, ids, rvecs, tvecs, controller):
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
        cv2.putText(frame, f"X:{x_cm:+.1f} Y:{y_cm:+.1f} Z(fwd):{z_cm:.1f} H:{horiz_cm:.1f}",
                    (10, 30 + 22*i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,0), 2)
    return frame

# ============================================================
# === Video Thread
# ============================================================

def video_thread(controller):
    print("Starting video thread...")
    frame_reader = controller.drone.get_frame_read()
    if frame_reader is None:
        print("Failed to get frame reader!")
        return

    alpha = 0.3

    while controller.is_running:
        frame = frame_reader.frame
        if frame is None or not hasattr(frame, "shape") or frame.size == 0:
            time.sleep(0.02)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rvecs, tvecs = detect_marker_pose(gray, controller)

        if ids is not None and rvecs is not None and tvecs is not None:
            # Compute horizontal (XZ-plane) distance
            x_cm = float(tvecs[0][0][0] * 100.0)
            z_cm = float(tvecs[0][0][2] * 100.0)
            horiz_cm = (x_cm**2 + z_cm**2) ** 0.5

            prev = controller.get_distance()
            if prev is None:
                controller.set_distance(horiz_cm)
            else:
                controller.set_distance((1 - alpha) * prev + alpha * horiz_cm)

            mc = np.mean(corners[0][0], axis=0)
            controller.set_marker_x(float(mc[0]))

            frame = draw_pose_axes(frame, corners, ids, rvecs, tvecs, controller)
            #print(f"Detected marker ID with opencv: {ids}")

            corner = np.array(corners[0], copy=True)
            corner.reshape((4,2))
            (top_left, top_right, bottom_right, bottom_left) = corner[0]
            marker_center_x = int((top_left[0]+bottom_right[0])//2)
            marker_center_y = int((top_right[1]+bottom_left[1])//2) 
            print(f"Marker center position: {marker_center_x}, {marker_center_y}")
            controller.set_marker_center(marker_center_x, marker_center_y)
        else:
            controller.set_distance(None)
            controller.set_marker_x(None)

        controller.set_frame(frame)

# ============================================================
# === Centering & Downward Landing
# ============================================================

def center_on_marker(controller):
    marker_x = controller.get_marker_x()
    frm = controller.get_frame()
    if marker_x is None or frm is None:
        print("Searching for marker...")
        controller.drone.send_rc_control(0, 0, 0, 10)
        return False

    frame_center_x = frm.shape[1] / 2.0
    x_error = marker_x - frame_center_x
    threshold = 20

    if abs(x_error) > threshold:
        yaw_speed = int(np.clip(x_error / 8.0, -25, 25))
        controller.drone.send_rc_control(0, 0, 0, yaw_speed)
        controller.is_centered= False
        return False
    else:
        if not controller.is_centered:
            print("Marker centered!")
            controller.drone.send_rc_control(0, 0, 0, 0)
            time.sleep(0.3)
            controller.is_centered = True
        return True

def downward_center_and_land(controller):
    print("\nSwitching to downward camera for smooth landing...")
    controller.drone.send_command_with_return("downvision 1")
    controller.using_downvision = True
    time.sleep(2)

    # --- basic parameters ---         # cm/s base RC speed
    center_tolerance = 8      # pixel tolerance for "centered"
    descend_speed = -20        # cm/s downward
    min_height = 20            # stop when below this      # largest dx, dy we expect
    kx = 0.6
    ky = 0.6
    def rc_pulse(lr=0, fb=0, ud=0, dur=0.3):
        controller.drone.send_rc_control(int(lr), int(fb), int(ud), 0)
        time.sleep(float(dur))
        controller.drone.send_rc_control(0, 0, 0, 0)

    while controller.is_running:
        frame = controller.get_frame() #Probably because the frame of the controller is still from the previous front camera
        if frame is None:
            time.sleep(0.1)
            continue
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rvecs, tvecs = detect_marker_pose(gray_frame, controller)
        
        marker_center = controller.get_marker_center()
        
        #logging.info(f"Downward centering loop: marker_center={marker_center} at {time.time()}")
        print(f"INFO: Downward centering loop: marker_center={marker_center} at {time.time()}")
        if rvecs is not None and tvecs is not None:
            fw, fh = frame.shape[1], frame.shape[0]
            dx = marker_center[0] - fw/2
            dy = marker_center[1] - fh/2

            x_offset, y_offset = tvecs[0][0][0]*100, tvecs[0][0][1]*100

            if abs(x_offset) < center_tolerance and abs(y_offset) < center_tolerance:
                print("Landing.")
                break
            
            else:
                delta_x = x_offset*kx
                delta_y = y_offset*ky

                lr = int(-delta_y)
                fb = int(-delta_x)

                print(f"Offset big (dx={dx:.0f}, dy={dy:.0f}), offset x in cm = {x_offset}, offset_y in cm = {y_offset} → "
                  f"lr={lr:.0f}, fb={fb:.0f}")
                    
                controller.drone.send_rc_control(lr, fb, 0, 0)
                time.sleep(0.2)
                controller.drone.send_rc_control(0,0,0,0)
                continue
        else: 
            controller.drone.send_rc_control(0,0,0,0)
            time.sleep(1)
            continue
    print("Centered — descending slightly...")
    rc_pulse(0, 0, descend_speed, 0)
    time.sleep(0.3)

    h = controller.drone.get_height()
    if h is not None and h <= min_height:
        print(f"Near ground (h={h} cm) — landing.")

    controller.drone.send_rc_control(0, 0, 0, 0)
    controller.drone.land()
    controller.movement_completed = True
    print("Landed safely.")

# ============================================================
# === Movement Thread
# ============================================================

def movement_thread(controller):
    print("Starting movement thread...")
    print("Taking off...")
    controller.drone.takeoff()
    controller.has_taken_off = True
    controller.drone.move_up(30)
    time.sleep(1)

    while controller.is_running and controller.get_distance() is None:
        time.sleep(0.1)

    initial_distance = controller.get_distance()
    if initial_distance is None:
        print("Could not detect marker for initial distance!")
        controller.drone.land()
        return

    print(f"Initial horizontal distance to marker: {initial_distance:.1f} cm")

    print("\nStep 1: Centering on marker...")
    while not center_on_marker(controller) and controller.is_running:
        time.sleep(0.1)
    time.sleep(0.6)

    print("\nStep 2: Gradual horizontal approach...")
    total_forward_distance = initial_distance*0.9
    num_segments = 4
    forward_per_segment = int(total_forward_distance / num_segments)

    try:
        current_altitude = controller.drone.get_height()  # in cm
    except Exception:
        current_altitude = 100  # fallback default if API fails
    print(f"Current altitude: {current_altitude} cm")
    target_altitude = 60

    total_descent = max(current_altitude - target_altitude, 0)
    descent_per_segment = int(total_descent / num_segments)

    for i in range(num_segments):
        if not controller.is_running:
            break
        if i != 0:
            forward_per_segment = controller.get_distance()/(num_segments-(i+1))
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

        print("Recentering on marker (if visible)...")
        max_center_attempts = 15  # about 1.5 seconds max

        for _ in range(max_center_attempts):
            if not controller.is_running:
                break

            # If marker_x is None, marker is lost → skip centering
            if controller.get_marker_x() is None:
                print("Marker lost — skipping re-centering.")
                break

            # Try to center if marker still visible
            if center_on_marker(controller):
                print("Marker centered.")
                break

            time.sleep(0.1)

        time.sleep(0.4)

    downward_center_and_land(controller)


# ============================================================
# === Display Loop
# ============================================================

def display_loop(controller):
    cv2.namedWindow("Tello Camera", cv2.WINDOW_NORMAL)
    while controller.is_running:
        frame = controller.get_frame()
        if frame is not None and frame.size:
            cv2.imshow("Tello Camera", frame)
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
    try:
        video_handler = threading.Thread(target=video_thread, args=(controller,), daemon=True)
        move_handler  = threading.Thread(target=movement_thread, args=(controller,), daemon=True)

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
    main()
