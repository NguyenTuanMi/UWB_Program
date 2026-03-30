import cv2
from cv2 import aruco
import numpy as np
import time
import threading
from UWB_Manipulation.UWB_Reader import get_target_position
from swarmserver.swarmserverclient_v2 import MarkerClient
from shared_utils.dronecontroller import *
from shared_utils.yaw_controller import RotationResult
import math
import json
import random

strafe_speed = 1.0
pi_id = 10
tag_id = 10

# ============================================================
# === Utility Functions
# ============================================================
def load_drone_info(filename='drones.json'):
    try:
        with open(filename, 'r') as file:
            return json.load(file)
    except FileNotFoundError:
        print(f"{filename} not found!")
        return []
    except json.JSONDecodeError:
        print(f"Error decoding JSON in {filename}!")
        return []

def obtain_network(tello_id: int, tello_info: dict):
    network_config = {}
    for drone in tello_info:
        if drone['id'] == tello_id:
            network_config['host'] = drone["TELLO_IP"]
            network_config['control_port'] = drone['TELLO_PORT']
            network_config['state_port'] = drone['LOCAL_PORT']
            network_config['video_port'] = drone['VIDEO_PORT']
            return network_config

# UWB thread
def uwb_poll_thread(tag_id, controller: DroneController):
    poll_dt = 0.1  # 5 Hz
    while controller.is_running:
        try:
            position = get_target_position(tag_id)  # meters from UWB_Reader
            if position and position != (0, 0, 0):
                x_cm, y_cm, z_cm = position[0]*100.0, position[1]*100.0, position[2]*100.0
                controller.set_latest_uwb((x_cm, y_cm, z_cm))
                # print(f"The latest UWB: {(x_cm, y_cm, z_cm)}")
        except Exception as exc:
            print(f"[UWB] poll error: {exc}")
        time.sleep(poll_dt)

def is_drone_flipped(controller: DroneController):
    try: 
        pitch = abs(controller.drone.get_pitch())
        roll = abs(controller.drone.get_roll())
        if pitch > 75 or roll > 75: 
            return True
    except Exception:
        pass
    return False

def watchdog_thread(controller: DroneController, markerclient: MarkerClient): 
    # initial_state = controller.drone.get_current_state()
    last_state = {}
    last_time = time.time()
    while controller.is_running:
        time.sleep(0.2)

        print(f"Current State: {controller.drone.get_current_state()}")

        if not controller.is_navigating or not controller.is_rotating:
            last_time = time.time()
            continue
        
        current_state = controller.drone.get_current_state()
        if current_state and current_state != last_state:
            last_time = time.time()
            last_state = current_state.copy()

        if (time.time() - last_time) > 4 or controller.drone.get_distance_tof() < 30 or is_drone_flipped(controller):
            with controller.waypoint_lock:
                if controller.current_waypont:
                    markerclient.send_update('waypoint', marker_id=controller.current_waypont(), detected=False)
                    if controller.current_waypont != 0:
                        markerclient.send_update('waypoint', marker_id=controller.current_waypont()  - 1, detected=False)
                    controller.is_running = False
                    controller.current_waypont = None
            controller.is_navigating = False 
            controller.is_running = False

def uwb_fly_to(controller: DroneController, target_x, target_y, arrival_threshold=15,
               max_speed=80, min_speed=15, slowdown_radius=200,
               loop_hz=10, timeout=60):
    """
    Fly to target using RC control with real-time UWB feedback.
    Every loop: poll UWB -> calculate -> send RC. No retries, no caching.

    Parameters:
    drone: Connected Tello object used to send RC commands.
    target_x: Target world X position in cm (UWB frame).
    target_y: Target world Y position in cm (UWB frame).
    arrival_threshold: Euclidean distance in cm considered "arrived".
    max_speed: Maximum RC speed command magnitude during cruise.
    min_speed: Minimum RC speed used by proportional slowdown.
    slowdown_radius: Distance in cm where proportional deceleration starts.
    loop_hz: Control-loop update rate (UWB poll + RC send) in Hz.
    timeout: Safety timeout in seconds before aborting this leg.
    """

    loop_period = 1.0 / loop_hz  # Seconds per control loop.
    deadline = time.time() + timeout  # Absolute timeout time.
    released_prev_wp = False
    best_dist = float('inf')
    leg_distance = None
    dynamic_max_speed = max_speed  # May be reduced for short legs.
    dynamic_slowdown_radius = slowdown_radius  # May be expanded for short legs.
    hard_brake_distance = max(35, arrival_threshold + 25)  # Enter aggressive braking zone.
    micro_correction_distance = max(arrival_threshold + 1, 7)  # Final low-speed correction zone.
    brake_pulse_done = False
    last_cmd_lr = 0
    last_cmd_fb = 0
    lateral_scale_far = 0.90  # LR damping factor far from target.
    lateral_scale_near = 0.62  # LR damping factor near target.
    lateral_limit_far = 68  # Max LR command far from target.
    lateral_limit_near = 26  # Max LR command near target.
    start_x = 0.0
    start_y = 0.0
    path_ux = 1.0
    path_uy = 0.0
    cte_gain = 0.5  # Cross-track correction gain.
    cte_deadband = 3.0  # Ignore tiny path error (cm).
    cte_cap = 22.0  # Max cross-track correction command.
    reverse_cte_boost = 1.35  # Extra path correction when moving backward.
    reverse_lr_boost = 1.18  # Extra LR authority when backward FB command is active.

    # print(f"[DRONE {drone_id}] uwb_fly_to RC: target=({target_x}, {target_y}), "
    #       f"threshold={arrival_threshold}cm, max_speed={max_speed}, slowdown={slowdown_radius}cm")

    # ── speed_adjust() — three-zone logarithmic profile ────────────────────
    # Zone 1 (dist > 300)  : scale = 1.0          → full cruise speed
    # Zone 2 (50–300 cm)   : scale = log10(d/30)  → smooth log decay
    #                         at d=300: log10(10)=1.0  (seamless join with zone 1)
    #                         at d=50 : log10(5/3)≈0.22 (smooth entry into zone 3)
    # Zone 3 (dist < 50)   : scale decays linearly 0.22→0.025 to prevent
    #                         abrupt step-drop and guarantee a non-None return.
    # Pre-compute constants so the inner loop has zero redundant work.
    _log_scale_at_50 = math.log10(50.0 / 30.0)   # ≈ 0.2218 — zone 2/3 join point
    def speed_adjust(distance):
        if distance > 300.0:
            return 1.0
        elif distance >= 50.0:
            return math.log10(distance / 30.0)     # 0.22 … 1.0
        else:
            # Linear taper from _log_scale_at_50 down to 0.025 at dist=0
            # Keeps the curve continuous and always returns a valid float.
            return 0.025 + (_log_scale_at_50 - 0.025) * (distance / 50.0)


    leg_start_time = time.perf_counter()  # timing: start of this leg

    try:
        while time.time() < deadline:

            # Poll UWB -> Calculate -> Send RC
            uwb_raw = controller.get_latest_uwb()
            if uwb_raw is None:
                time.sleep(0.05)
                continue
            cur_x = uwb_raw[0] * 100
            cur_y = uwb_raw[1] * 100

            dx_world = target_x - cur_x
            dy_world = target_y - cur_y
            dist = math.sqrt(dx_world**2 + dy_world**2)

            if leg_distance is None:
                start_x = cur_x
                start_y = cur_y
                leg_distance = dist

                if leg_distance > 1:
                    path_ux = (target_x - start_x) / leg_distance
                    path_uy = (target_y - start_y) / leg_distance
                else:
                    path_ux, path_uy = 1.0, 0.0

                # Keep short hops from overshooting too aggressively.
                if leg_distance < 120:
                    dynamic_max_speed = min(max_speed, 55)
                    dynamic_slowdown_radius = max(slowdown_radius, int(leg_distance * 0.95))
                elif leg_distance < 220:
                    dynamic_max_speed = min(max_speed, 68)
                    dynamic_slowdown_radius = max(slowdown_radius, int(leg_distance * 0.8))
                else:
                    dynamic_max_speed = max_speed
                    dynamic_slowdown_radius = slowdown_radius

                # print(f"[DRONE {drone_id}] RC profile (LOG): leg={leg_distance:.0f}cm "
                #       f"max={dynamic_max_speed} slowdown={dynamic_slowdown_radius} "
                #         f"hard_brake={hard_brake_distance} micro={micro_correction_distance} "
                #         f"cte_gain={cte_gain}")

            if dist < best_dist:
                best_dist = dist

            # Arrival check
            if dist <= arrival_threshold:
                src_lr = last_cmd_lr
                src_fb = last_cmd_fb
                brake_lr = int(max(-52, min(52, -1.30 * src_lr)))
                brake_fb = int(max(-50, min(50, -1.35 * src_fb)))
                if brake_lr != 0 or brake_fb != 0:
                    controller.drone.send_rc_control(brake_lr, brake_fb, 0, 0)
                    time.sleep(0.35)
                controller.drone.send_rc_control(0, 0, 0, 0)
                leg_elapsed = time.perf_counter() - leg_start_time
                # print(f"[DRONE {drone_id}] uwb_fly_to RC: ARRIVED - dist={dist:.1f}cm "
                #       f"pos=({cur_x:.0f},{cur_y:.0f}) time={leg_elapsed:.2f}s")
                break

            # Release previous waypoint once
            

            # ── speed_adjust() three-zone logarithmic scaling ───────────────
            # Returns a scale in (0, 1] that is applied between min_speed and
            # dynamic_max_speed.  The two-stage hard-brake block below can
            # still override this for the final approach metres.
            speed = min_speed + (dynamic_max_speed - min_speed) * speed_adjust(dist)

            # Two-stage braking: keep speed healthier in outer brake zone,
            # then brake very hard in the final inner zone to avoid long crawl.
            if dist < hard_brake_distance:
                inner_brake_distance = max(arrival_threshold + 10, 14)
                if dist > inner_brake_distance:
                    outer_span = max(1.0, hard_brake_distance - inner_brake_distance)
                    outer_ratio = max(0.0, min(1.0, (dist - inner_brake_distance) / outer_span))
                    hard_brake_speed = 18 + (dynamic_max_speed - 18) * (outer_ratio ** 1.30)
                else:
                    inner_ratio = max(0.0, min(1.0, dist / inner_brake_distance))
                    hard_brake_speed = 6 + (14 - 6) * (inner_ratio ** 4.6)
                speed = min(speed, hard_brake_speed)

            # Tiny final corrections near target.
            if dist < micro_correction_distance:
                speed = min(speed, 8)
            if dist < arrival_threshold + 1:
                speed = min(speed, 6)

            speed = max(4, min(dynamic_max_speed, speed))

            ux = dx_world / dist
            uy = dy_world / dist
            vx_world = ux * speed
            vy_world = uy * speed
            heading = controller.get_curent_heading()
            hdg_rad = math.radians(heading)

            # Straight-line correction during cruise: pull back to start->target path.
            if dist > hard_brake_distance:
                nx = -path_uy
                ny = path_ux
                cte = ((cur_x - start_x) * nx) + ((cur_y - start_y) * ny)
                if abs(cte) > cte_deadband:
                    cte_excess = abs(cte) - cte_deadband
                    cte_corr = max(-cte_cap, min(cte_cap, -math.copysign(cte_excess * cte_gain, cte)))

                    # Backward travel usually needs stronger line-holding correction.
                    fwd_component = vy_world * math.cos(hdg_rad) + vx_world * math.sin(hdg_rad)
                    if fwd_component < 0:
                        cte_corr *= reverse_cte_boost

                    vx_world += nx * cte_corr
                    vy_world += ny * cte_corr

            # World -> body frame
            x_body = vx_world * math.cos(hdg_rad) - vy_world * math.sin(hdg_rad)
            y_body = vy_world * math.cos(hdg_rad) + vx_world * math.sin(hdg_rad)

            # Lateral (LR) axis tends to overshoot more than FB; damp and cap it.
            if dist < dynamic_slowdown_radius:
                near_ratio = 1.0 - max(0.0, min(1.0, dist / dynamic_slowdown_radius))
            else:
                near_ratio = 0.0
            lateral_scale = lateral_scale_far - (lateral_scale_far - lateral_scale_near) * near_ratio
            lateral_limit = int(round(lateral_limit_far - (lateral_limit_far - lateral_limit_near) * near_ratio))
            x_body *= lateral_scale

            lr = int(max(-lateral_limit, min(lateral_limit, x_body)))
            fb = int(max(-100, min(100, y_body)))

            if fb < -8:
                lr = int(max(-lateral_limit, min(lateral_limit, lr * reverse_lr_boost)))

            if dist < hard_brake_distance:
                lr = int(max(-12, min(12, lr)))
                fb = int(max(-16, min(16, fb)))
            if dist < micro_correction_distance:
                lr = int(max(-7, min(7, lr)))
                fb = int(max(-9, min(9, fb)))

            # One-shot reverse brake pulse when entering hard-brake zone.
            if (not brake_pulse_done
                and dist < hard_brake_distance
                and dist > micro_correction_distance):
                src_lr = last_cmd_lr if (last_cmd_lr != 0 or last_cmd_fb != 0) else lr
                src_fb = last_cmd_fb if (last_cmd_lr != 0 or last_cmd_fb != 0) else fb
                brake_lr = int(max(-55, min(55, -1.25 * src_lr)))
                brake_fb = int(max(-48, min(48, -1.20 * src_fb)))#valuces for forward and backward braking may need tweaking depending on how the drone responds
                if brake_lr != 0 or brake_fb != 0:
                    controller.drone.send_rc_control(brake_lr, brake_fb, 0, 0)
                    time.sleep(0.20)
                    # print(f"[DRONE {drone_id}] Brake pulse: lr={brake_lr} fb={brake_fb} at dist={dist:.1f}cm")
                brake_pulse_done = True

            controller.drone.send_rc_control(lr, fb, 0, 0)
            last_cmd_lr, last_cmd_fb = lr, fb

            if int(time.time() * loop_hz) % (loop_hz * 2) == 0:
                print(f"[DRONE {pi_id}] RC: dist={dist:.0f}cm best={best_dist:.0f}cm "
                      f"speed={speed:.0f} lr={lr} fb={fb} hdg={heading} "
                      f"pos=({cur_x:.0f},{cur_y:.0f})")

            time.sleep(loop_period)
        else:
            controller.drone.send_rc_control(0, 0, 0, 0)
            print(f"[DRONE {pi_id}] uwb_fly_to RC: TIMEOUT after {timeout}s")

    except Exception:
        controller.drone.send_rc_control(0, 0, 0, 0)
        raise

    if not released_prev_wp:
                print(f"RELEASING {controller.get_current_waypoint()-1} (departing)")
                controller.marker_client.send_update('waypoint', marker_id=controller.get_current_waypoint()-1, detected=False)
                released_prev_wp = True    
    return True


def rc_move_to(controller: DroneController, target_xy_cm, pos_thresh_cm=35, max_rc=100, min_rc=10):
    """
    Drive using send_rc_control until within pos_thresh_cm of target_xy_cm.
    target_xy_cm: (x_cm, y_cm) in world coords.
    """
    print(f"[RC] Navigating to {target_xy_cm}")
    
    while controller.is_running:
        uwb = controller.get_latest_uwb()
        if uwb is None:
            controller.drone.send_rc_control(0, 0, 0, 0)
            time.sleep(0.1)
            continue

        cur_xy = (uwb[0], uwb[1])
        dx = target_xy_cm[0] - cur_xy[0]
        dy = target_xy_cm[1] - cur_xy[1]
        dist = math.hypot(dx, dy)

        if dist <= pos_thresh_cm:
            controller.drone.send_rc_control(0, 0, 0, 0)
            print(f"[RC] target reached within {dist:.1f} cm")
            return True
        
        heading = controller.get_heading()
        print(f"Drone heading: {heading}")
        h = math.radians(heading)
        
        # Clarified projection math (Dot Products)
        err_body_fwd = dx * math.sin(h) + dy * math.cos(h)
        err_body_right = dx * math.cos(h) - dy * math.sin(h)

        # Normalize the vectors
        direction_fwd = err_body_fwd / dist 
        direction_right = err_body_right / dist
        
        # BUG FIX: Ramp down within 300cm (3.0m), not 3.0cm
        speed_scale = min(1.0, dist / 300.0)  
        
        # BUG FIX: Apply max_rc so the drone doesn't fly at max speed
        speed_multiplier = max(min_rc, max_rc * speed_scale)
        
        lr = int(direction_right * speed_multiplier)
        if 0 < abs(lr) < 8:
            lr = -8 if lr < 0 else 8

        fb = int(direction_fwd * speed_multiplier)
        if 0 < abs(fb) < 8:
            fb = -8 if fb < 0 else 8
        
        controller.drone.send_rc_control(lr, fb, 0, 0)
        time.sleep(0.1)

    controller.drone.send_rc_control(0, 0, 0, 0)
    time.sleep(0.1)
    print("[RC] move timed out or stopped.")
    return False

def get_distance_with_retry(controller: DroneController, id, max_attempts=30):
    for attempt in range(max_attempts):
        d = controller.get_distance(id)
        print(f"Current distance to marker {id}: {d}")
        if d is not None:
            return d
        time.sleep(0.1)
    return None

def get_visible_marker_list_with_retry(controller: DroneController, max_attempts=5):
    for attempt in range(max_attempts):
        d = controller.get_current_visible_snapshot()
        print(f"Current visible snapshot: {d}")
        if d is not None and d != {}:
            return d
        time.sleep(0.1)
    return {}

def get_marker_x_with_retry(controller: DroneController, id, max_attempts=10):
    for attempt in range(max_attempts):
        d = controller.get_marker_x(id)
        if d is not None:
            return d
        time.sleep(0.1)
    return None

def get_calibration_parameters(controller: DroneController):
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

def detect_marker_pose(gray_frame, controller: DroneController,
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

    try:
        rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(corners, marker_size_m, K, D)
    except cv2.error:
        return corners, ids, None, None
    #ontroller.set_rtvec(rvecs, tvecs)
    return corners, ids, rvecs, tvecs

def draw_pose_axes(frame, corners, ids, rvecs, tvecs, controller: DroneController, is_fire):
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
        
        # controller.set_rtvec(x_cm, y_cm)
        color = (0,255,0)
        if is_fire:
            color = (255, 0, 0)
        cv2.putText(frame, f"X:{x_cm:+.1f} Y:{y_cm:+.1f} Z(fwd):{z_cm:.1f} H:{horiz_cm:.1f}",
                    (10, 30 + 22*i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    return frame

# ============================================================
# === Video Thread
# ============================================================

def video_thread(controller: DroneController):
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

        new_visible = {}
        fire_visible = {}
        victim_visible = {}

        if ids is not None and rvecs is not None and tvecs is not None:
            has_interrupt_target = False
            for i in range(len(ids)):
                if ids[i][0] == 0 or ids[i][0] not in controller.total_marker:
                    continue

                x_cm = float(tvecs[i][0][0] * 100.0)
                z_cm = float(tvecs[i][0][2] * 100.0)

                # Compute horizontal (XZ-plane) distance
                horiz_cm = (x_cm**2 + z_cm**2) ** 0.5

                if horiz_cm > 600:
                    continue

                is_fire = ids[i][0] in controller.invalid_ids
                is_victim = ids[i][0] in controller.valid_ids or ids[i][0] in controller.bonus_victims

                if not is_fire and not is_victim:
                    continue

                if controller.is_scanning and ids[i][0] not in controller.scan_ignore_marker:
                    controller.scan_ignore_marker.add(ids[i][0]) # Never stop for this one again THIS scan
                    has_interrupt_target = True
            
                prev = controller.get_distance(ids[i][0])
                if prev is None:
                    controller.set_distance(horiz_cm, ids[i][0])
                else:
                    controller.set_distance((1 - alpha) * prev + alpha * horiz_cm, ids[i][0])
                    # mc = np.mean(corners[0][0], axis=0)
                    
                controller.set_marker_x(x_cm, ids[i][0])

                # Update persistent discovery sets
                if is_fire and ids[i][0] not in fire_visible:
                    fire_visible[ids[i][0]] = {
                    'x': x_cm,
                    'is_fire': is_fire
                    }
                elif is_victim and ids[i][0] not in victim_visible:
                    victim_visible[ids[i][0]] = {
                    'x': x_cm,
                    'is_fire': is_fire
                    }
                frame = draw_pose_axes(frame, corners, ids, rvecs, tvecs, controller, is_fire)  
            new_visible = fire_visible | victim_visible
            
            if has_interrupt_target:
                controller.interrupt_scan_event.set()

        if tvecs is not None:
            controller.set_rtvec(rvecs, tvecs)
        else: 
            controller.set_rtvec(None, None)

        with controller.current_visible_lock:
            controller.current_visible = new_visible

        controller.set_frame(frame)
        time.sleep(0.033)

# ============================================================
# === Centering & Downward Landing
# ============================================================

def center_on_marker(controller: DroneController, id, marker_client: MarkerClient):
    marker_client.send_update('marker', marker_id=int(id), detected=True)
    controller.set_marker_x(None, id)
    time.sleep(0.2)
    marker_x = get_marker_x_with_retry(controller, id) 
    frm = controller.get_frame()
    if marker_x is None or frm is None:
        print("Marker not exist. Proceed to the next step")
        controller.drone.send_rc_control(0, 0, 0, 0)
        return True

    x_error = marker_x
    threshold = 15
    k_p = 0.2
    if abs(x_error) > 300:
        k_p = 0.12
    if abs(x_error) > threshold:
        raw_yaw = int(x_error * k_p)
        yaw_speed = max(8, abs(raw_yaw)) * (1 if raw_yaw >= 0 else -1)
        controller.drone.send_rc_control(0, 0, 0, yaw_speed)
        controller.is_centered= False
        marker_client.send_update('marker', marker_id=int(id), detected=True)
        return False
    else:
        if not controller.is_centered:
            print("Marker centered!")
            controller.drone.send_rc_control(0, 0, 0, 0)
            time.sleep(0.1)
            controller.is_centered = True
        return True

def downward_center_and_land(controller: DroneController, target_marker_id: int, marker_client: MarkerClient):
    bonus_detected = False
    if target_marker_id in controller.bonus_victims:
        bonus_detected = True
    marker_client.send_update('marker', marker_id=int(target_marker_id), detected=True, bonus_detected=bonus_detected)
    # marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True, bonus_detected=bonus_detected)
    for marker_id, info in controller.get_current_visible_snapshot().items():
        if marker_id != target_marker_id:
            marker_client.send_update('marker', marker_id=int(marker_id), detected=False)
    
    print("\nSwitching to downward camera for smooth landing...")
    for i in range(3):
        controller.drone.send_command_with_return("downvision 1")
        time.sleep(0.1)
    controller.using_downvision = True

    # --- basic parameters ---         # cm/s base RC speed
    center_tolerance = 8      # pixel tolerance for "centered"
    descend_speed = -20        # cm/s downward
    min_height = 60            # stop when below this      # largest dx, dy we expect
    kx = 0.6
    ky = 0.6

    marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True, bonus_detected=bonus_detected)
    # marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True, bonus_detected=bonus_detected)
    count = 0
    while controller.is_running:
        frame = controller.get_frame() #Probably because the frame of the controller is still from the previous front camera
        # marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True, bonus_detected=bonus_detected)
        if frame is None:
            time.sleep(0.05)
            continue
        # gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # corners, ids, rvecs, tvecs = detect_marker_pose(gray_frame, controller)
        
        #marker_center = controller.get_marker_center()
        # marker_client.send_update('marker', marker_id=int(target_marker_id), detected=True, bonus_detected=bonus_detected)
        # marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True, bonus_detected=bonus_detected)
        # for marker_id in marker_list:
        #     if marker_id != target_marker_id:
        #         marker_client.send_update('marker', marker_id=int(marker_id), detected=False)
        #   logging.info(f"Downward centering loop: marker_center={marker_center} at {time.time()}")
        rvecs, tvecs = controller.get_rtvec()
        print(f"INFO: Downward centering loop at {time.time()}")
        if rvecs is not None and tvecs is not None:
            count = 0
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
            # controller.drone.send_rc_control(0,0,0,0)
            # time.sleep(0.2)
            # count += 1
            # if count > 4:
            break
    print("Centered — descending slightly...")
    controller.drone.send_rc_control(0, 0, descend_speed, 0)
    time.sleep(0.3)
    controller.drone.send_rc_control(0, 0, 0, 0)

    h = controller.drone.get_height()
    if h is not None and h <= min_height:
        print(f"Near ground (h={h} cm) — landing.")
    if bonus_detected:
        pos = get_target_position(controller.drone_uwbtag)
        marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True, bonus_detected=bonus_detected, bonus_position=pos)
    else:
        marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True, bonus_detected=bonus_detected)
    controller.drone.send_rc_control(0, 0, 0, 0)
    controller.drone.land()
    controller.movement_completed = True
    print("Landed safely.")

def scan_for_marker(controller: DroneController, marker_client: MarkerClient, searching_for_fire = False):
    # controller.set_is_rotating(True)
    controller.scan_ignore_marker.clear()
    time.sleep(0.3)
    rotation = 0
    target_rotation = 360
    rotating_angle = 60
    offset = 0
    controller.is_rotating = True
    while rotation < target_rotation:
        visible = get_visible_marker_list_with_retry(controller)
        
        for id, info in visible.items():
            if searching_for_fire:
                if id not in controller.invalid_ids:
                    continue

            claim_success = marker_client.send_update('marker', marker_id=int(id), detected=True)
            if not claim_success:
                print(f"[COLLISION AVOIDED] Marker {id} is already taken by another drone!")
                continue 
            print(f"Detecting marker {id}")
            
            controller.set_distance(None, id)
            time.sleep(0.1)
            dist = get_distance_with_retry(controller, id, max_attempts=3)

            if dist is None:
                marker_client.send_update('marker', marker_id=int(id), detected=False)
                continue
            
            # marker_client.send_update('marker', marker_id=int(id), detected=True)
            controller.drone.send_rc_control(0, 0, 0, 0)
            print(f"Visible marker {id} at distance {dist:.1f}cm, is_fire={info['is_fire']}")
            approached = locate_marker(controller, id, marker_client, is_fire=info['is_fire'])
            if approached:
                return

        # Enable the interrupt flag for the video_thread
        controller.is_scanning = True
        controller.interrupt_scan_event.clear()
        time.sleep(0.1)

        result, deg_turned = controller.yaw_controller.yaw_right_by_angle(rotating_angle - offset)

        controller.is_scanning = False
         # Accurately track progress!
        if result == RotationResult.INTERRUPTED:
            print("[SCAN] Rotation interrupted by marker — re-reading snapshot")
            if abs(deg_turned - 60) > 15:
                offset = deg_turned
                rotation += offset
                continue
            offset = 0
            # continue  # loop back, read visible again WITHOUT rotating
        elif result in (RotationResult.ABORTED, RotationResult.TIMEOUT):
            print(f"[SCAN] Rotation ended with {result} — stopping scan")
            return
        rotation += 60
        controller.is_rotating = False
        time.sleep(0.5)
    # This part is to align the drone back to the original course heading
    heading = controller.get_heading()
    print(f"Current heading: {heading}")
    
def locate_marker(controller: DroneController, id, marker_client: MarkerClient, is_fire: bool):
    claimed = marker_client.send_update('marker', marker_id=int(id), detected=True)
    if not claimed: 
        print(f"[COLLISION AVOIDED] Marker {id} is already taken by another drone!")
        return False
    
    print(f"\nEntering locating marker stage for id {id}")
    
    print("\nStep 1: Centering on marker...")
    while not center_on_marker(controller, id, marker_client) and controller.is_running:
        marker_client.send_update('marker', marker_id=int(id), detected=True)
        time.sleep(0.1)
    
    controller.set_distance(None, id)
    time.sleep(0.2)
    initial_distance = get_distance_with_retry(controller, id)
    must_center = False
    
    if initial_distance is None:
        # marker_client.send_update('marker', marker_id=int(id), claimed=False)
        print(f"Marker {id} was lost")
        status = f"Marker {id} was lost"
        if id in controller.valid_ids:          
            print("Could not detect marker for initial distance!")
        elif id in controller.invalid_ids:
            print(f"Could not detect fire marker {id} for initial distance!")
        return False
    marker_client.send_update('waypoint', marker_id=controller.get_current_waypoint(), detected=False)
    if id in controller.bonus_victims:
        curr_pose = controller.get_latest_uwb()
        heading = controller.get_heading()
        target_xy = (
            (curr_pose[0] + initial_distance * math.sin(math.radians(heading)))/100,  # x increment
            (curr_pose[1] + initial_distance * math.cos(math.radians(heading)))/100  # y increment
        )
        marker_client.send_update('marker', marker_id=int(id), detected=True, bonus_detected=True, bonus_position=list(target_xy))
    # if initial_distance > 350:
    #     must_center = True
    
    print(f"Initial horizontal distance to marker: {initial_distance:.1f} cm")

    print("\nStep 2: Gradual horizontal approach...")
    total_forward_distance = initial_distance*0.9
    num_segments = 2
    initial_segment = int(total_forward_distance / num_segments)
    forward_per_segment = initial_segment

    try:
        current_altitude = controller.drone.get_height()  # in cm
    except Exception:
        current_altitude = 100  # fallback default if API fails
    print(f"Current altitude: {current_altitude} cm")
    # target_altitude = 60

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
                # print("Recentering on marker (if visible)...")
                # while not center_on_marker(controller, id, marker_client) and controller.is_running:
                #     marker_client.send_update('marker', marker_id=int(id), detected=True)
                #     time.sleep(0.1)
                dist = get_distance_with_retry(controller, id, max_attempts=5)
                forward_per_segment = int(dist*0.9/(num_segments-(i)))
        
        print(f"\nExecuting segment {i+1}/{num_segments}")
        curr_pose = controller.get_latest_uwb()
        heading = controller.get_heading()
        target_xy = (
            curr_pose[0] + forward_per_segment * math.sin(math.radians(heading)),  # x increment
            curr_pose[1] + forward_per_segment * math.cos(math.radians(heading)),  # y increment
        )
        print(f"The target position: {target_xy}")
        rc_move_to(controller, target_xy_cm=target_xy)
        # uwb_fly_to(controller, target_xy[0], target_xy[1])
        
        marker_client.send_update('marker', marker_id=int(id), detected=True)
        brake(controller.drone, 300)
        
        # descent_velocity = descent_per_segment/0.5
        # controller.drone.send_rc_control(0, 0, -int(descent_velocity), 0)  # gentle down velocity (cm/s)
        # time.sleep(0.2)
        # brake(controller.drone, 300)

        # if must_center:
        #     print("Recentering on marker (if visible)...")
        #     while not center_on_marker(controller, id, marker_client) and controller.is_running:
        #         time.sleep(0.1) 
    if not is_fire:
        scan_for_marker(controller, marker_client, searching_for_fire=True)
    downward_center_and_land(controller, id, marker_client)
    return True

def movement_thread(controller: DroneController, marker_client: MarkerClient):
    print("Starting movement thread...")
    group_id = 2
    
    print(f"Group id: {group_id}")
    uwb_raw = (0,0,0)
    while uwb_raw == (0,0,0):
        uwb_raw = get_target_position(tag_id)
    
    uwb_pos = [uwb_raw[0]*100, uwb_raw[1]*100]
    print(f" Ground pos: {uwb_pos}")
    
    controller.start_pose = uwb_pos

    # marker_client.client_takeoff_simul([99], f'Battery: {controller.drone.get_battery()}')
    marker_client.client_takeoff_simul([99], f'Battery: {controller.drone.get_battery()} - Start Pose: {controller.start_pose}')
    print("Taking off...")
    controller.drone.takeoff()
    controller.has_taken_off = True
    execute_waypoints(controller, marker_client, group_id)

def validate_waypoints(group_id):
    if group_id == 1:
        with open('waypoint6.json', 'r') as f:
            data = json.load(f)
    else: 
        with open('waypoint7.json', 'r') as f:
            data = json.load(f)
    
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

def execute_waypoints(controller: DroneController, marker_client: MarkerClient, group_id: int):
    waypoint_id = 0
    try:
        if group_id == 1:
            with open('waypoint6.json', 'r') as f:
                data = json.load(f)
        else:
            with open('waypoint7.json', 'r') as f:
                waypoint_id = 4
                data = json.load(f) 
        # waypoint_id = 0
        # Execute each waypoint
        for wp in data['wp']:
            # controller.set_current_waypoint(waypoint=waypoint_id)
            controller.is_navigating = True
            # Handle rotation
            status = "Orienting"

            # scan_marker(controller, marker_client=marker_client) #Check the availability of the marker_client
            time.sleep(random.uniform(0, 0.5))
            print(f"Starting waypoint: {waypoint_id}")
            if waypoint_id == 0:
                start_time = time.time()
                while time.time() - start_time < 15:
                    controller.drone.send_rc_control(0, 0, 0, 0)
                    time.sleep(0.5)
            while marker_client.send_update('waypoint', waypoint_id, detected=True) is False:
                print(f"waiting for waypoint {waypoint_id} to be available")
                controller.drone.send_rc_control(0, 0, 0, 0)
                time.sleep(0.5)
            
            if waypoint_id != 0:
                print(f"Release the waypoints: {waypoint_id - 1}")
                marker_client.send_update('waypoint', marker_id=waypoint_id-1, detected=False)

            status = "Proceeding forward"
            try:
                target_xy = (
                wp['position_cm']['x'],  # x increment
                wp['position_cm']['y'],  # y increment
                )
                print(f"The target position: {target_xy}")
                rc_move_to(controller, target_xy_cm=target_xy)
                #uwb_fly_to(controller, target_xy[0], target_xy[1])
                controller.is_navigating = False
            except:
                waypoint_id -= 1
                continue
            
            print(f"Drone at Waypoint {waypoint_id}")
            marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
            controller.set_current_waypoint(waypoint_id)
            if group_id == 1:
                if controller.prioritize_state:
                    if waypoint_id > 2:
                        scan_for_marker(controller, marker_client)
                else:
                    scan_for_marker(controller, marker_client)
            elif group_id == 2: 
                if waypoint_id in [4, 6, 8, 9]:
                        scan_for_marker(controller, marker_client)
            waypoint_id += 1
    
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

def display_loop(controller: DroneController):
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
    TELLO_INFO = load_drone_info()
    network_config = obtain_network(pi_id, TELLO_INFO)
    controller = DroneController(pi_id=pi_id, tag_id=tag_id, network_config=network_config, priority=True)
    markerclient = MarkerClient(drone_id=controller.drone_id)
    try:
        uwb_thread = threading.Thread(target=uwb_poll_thread, args=(controller.drone_uwbtag, controller), daemon=True)
        video_handler = threading.Thread(target=video_thread, args=(controller,), daemon=True)
        move_handler  = threading.Thread(target=movement_thread, args=(controller,markerclient,), daemon=True)
        watchdog_handler = threading.Thread(target=watchdog_thread, args=(controller, markerclient), daemon=True)

        video_handler.start()
        time.sleep(1.0)
        uwb_thread.start()
        move_handler.start()
        display_loop(controller)

        controller.is_running = False
        video_handler.join(timeout=2)
        move_handler.join(timeout=2)
        uwb_thread.join(timeout=2)
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
    group_id = 2
    if validate_waypoints(group_id=group_id):
        print("Validation passed. Starting execution...")
        main()
    else:
        print("Validation failed. Please check warnings above.")