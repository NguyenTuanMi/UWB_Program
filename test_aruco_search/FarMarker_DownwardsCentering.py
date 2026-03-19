import cv2
from cv2 import aruco
import numpy as np
from djitellopy import Tello
import time
import threading
from threading import Lock
import logging

#from exploring_drones import RC_SPEED_SCALE

# ============================================================
# === Drone Controller Class
# ============================================================
RC_SPEED_SCALE = 0.2
class DroneController:
    def __init__(self):
        self.drone = initialize_drone()
        self.frame = None
        self.frame_lock = Lock()
        self.distance = None
        self.distance_lock = Lock()
        self.marker_x = None
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

    def get_frame(self):
        with self.frame_lock:
            return self.frame.copy() if self.frame is not None else None

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
    drone = Tello()
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

def get_calibration_parameters():
    K = np.array([
        [473.11891765, 0.000000, 323.13378419],
        [0.000000, 475.40414375, 234.45123335],
        [0.000000, 0.000000, 1.000000]
    ])
    D = np.array([0.04172334, 0.38719427, -0.00921904, 0.00348033, -0.70063595])
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
    K, D = get_calibration_parameters()
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

    return valid_corners, np.array(valid_ids), rvecs, tvecs

def draw_pose_axes(frame, corners, ids, rvecs, tvecs):
    if ids is None or rvecs is None or tvecs is None:
        return frame
    K, D = get_calibration_parameters()
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

            frame = draw_pose_axes(frame, corners, ids, rvecs, tvecs)
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


def process_downward_frame(frame):
    if frame is None or frame.size == 0:
        return None, None
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_100)
    params = aruco.DetectorParameters()
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=params)
    if ids is not None:
        center = np.mean(corners[0][0], axis=0)
        return center, frame
    return None, frame

def downward_center_and_land(controller):
    print("\nSwitching to downward camera for smooth landing...")
    controller.drone.send_command_with_return("downvision 1")
    controller.using_downvision = True
    time.sleep(2)

    # --- basic parameters ---
    kx = 0.4
    ky = 0.4
    base_speed = 20            # cm/s base RC speed
    center_tolerance = 10      # pixel tolerance for "centered"
    descend_speed = -20        # cm/s downward
    min_height = 20            # stop when below this
    max_offset_px = 200        # largest dx, dy we expect

    def rc_pulse(lr=0, fb=0, ud=0, dur=0.3):
        controller.drone.send_rc_control(int(lr), int(fb), int(ud), 0)
        time.sleep(float(dur))
        controller.drone.send_rc_control(0, 0, 0, 0)

    while controller.is_running:
        frame = controller.get_frame() #Probably because the frame of the controller is still from the previous front camera
        if frame is None:
            time.sleep(0.1)
            continue

        marker_center, _ = process_downward_frame(frame)
        #logging.info(f"Downward centering loop: marker_center={marker_center} at {time.time()}")
        print(f"INFO: Downward centering loop: marker_center={marker_center} at {time.time()}")
        if marker_center is None:
            print("Marker lost — performing local search...")

            search_moves = [
                (-20, 0, 0, 0.3),  # left
                (20, 0, 0, 0.3),   # right
                (0, 20, 0, 0.3),   # forward
                (0, -20, 0, 0.3)   # backward
            ]

            found = False
            for lr, fb, ud, dur in search_moves:
                rc_pulse(lr, fb, ud, dur=2)
                time.sleep(0.5)  # short pause to stabilize

                # check if marker reappears
                frame = controller.get_frame()
                if frame is not None:
                    new_center, _ = process_downward_frame(frame)
                    if new_center is not None:
                        print("Marker re-acquired! Resuming centering...")
                        found = True
                        marker_center = new_center
                        break

            if not found:
                print("Marker still not found — hovering...")
                rc_pulse(0, 0, 0, 0.2)
                continue


        fw, fh = frame.shape[1], frame.shape[0]
        dx = marker_center[0] - fw//2
        dy = marker_center[1] - fh//2

        # normalize offset magnitude (0–1)
        norm_x = min(abs(dx) / max_offset_px, 1.0)
        norm_y = min(abs(dy) / max_offset_px, 1.0)

        # scale movement duration and speed
        move_time = 0.3 + 0.4 * max(norm_x, norm_y)  # 0.3–0.7 s
        #move_speed = base_speed * (0.5 + max(norm_x, norm_y))  # 10–30 cm/s
        #move_time = 0.4
        if abs(dx) > center_tolerance or abs(dy) > center_tolerance:
            lr = int(-dx * RC_SPEED_SCALE)
            fb = int(-dy * RC_SPEED_SCALE)
            lr = int(np.clip(lr, -30, 30))
            fb = int(np.clip(fb, -30, 30))

            print(f"Offset big (dx={dx:.0f}, dy={dy:.0f}) → "
                  f"lr={lr:.0f}, fb={fb:.0f}, time={move_time:.1f}s")
                    
            controller.drone.send_rc_control(lr, -fb, 0, 0)
            time.sleep(move_time)
            continue

        print("Centered — descending slightly...")
        rc_pulse(0, 0, descend_speed, 0)
        time.sleep(0.3)

        h = controller.drone.get_height()
        if h is not None and h <= min_height:
            print(f"Near ground (h={h} cm) — landing.")
            break

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
    num_segments = 2
    forward_per_segment = int(total_forward_distance / num_segments)

    try:
        current_altitude = controller.drone.get_height()  # in cm
    except Exception:
        current_altitude = 100  # fallback default if API fails
    print(f"Current altitude: {current_altitude} cm")
    target_altitude = 30

    total_descent = max(current_altitude - target_altitude, 0)
    descent_per_segment = int(total_descent / num_segments)

    for i in range(num_segments):
        if not controller.is_running:
            break
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
