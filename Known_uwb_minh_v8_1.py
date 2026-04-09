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
pi_id = 9
tag_id = 21

network_config = {
            'host': f'192.168.1.{100+pi_id}',     
            'control_port': 9000 + pi_id,
            'state_port': 8000 + pi_id,
            'video_port': 11100 + pi_id   
            }

group_1 = [9,6,19,10]
uwb_ground_height = 60

# ============================================================
# === Utility Functions
# ============================================================

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
        if abs(lr) < 8:
            if lr < 0:
                lr = -8
            else: 
                lr = 8

        fb = int(direction_fwd * speed_multiplier)
        if abs(fb) < 8:
            if fb < 0:
                fb = -8
            else: 
                fb = 8
        
        controller.drone.send_rc_control(lr, fb, 0, 0)
        time.sleep(0.1)

    controller.drone.send_rc_control(0, 0, 0, 0)
    time.sleep()
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
       
        with controller.current_visible_lock:
            controller.current_visible = new_visible

        controller.set_frame(frame)

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
    marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True, bonus_detected=bonus_detected)
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

    marker_client.send_update('marker', marker_id=int(target_marker_id), detected=True, bonus_detected=bonus_detected)
    marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True, bonus_detected=bonus_detected)
    while controller.is_running:
        frame = controller.get_frame() #Probably because the frame of the controller is still from the previous front camera
        marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True, bonus_detected=bonus_detected)
        if frame is None:
            time.sleep(0.05)
            continue
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rvecs, tvecs = detect_marker_pose(gray_frame, controller)
        
        #marker_center = controller.get_marker_center()
        marker_client.send_update('marker', marker_id=int(target_marker_id), detected=True, bonus_detected=bonus_detected)
        marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True, bonus_detected=bonus_detected)
        # for marker_id in marker_list:
        #     if marker_id != target_marker_id:
        #         marker_client.send_update('marker', marker_id=int(marker_id), detected=False)
        #   logging.info(f"Downward centering loop: marker_center={marker_center} at {time.time()}")
        print(f"INFO: Downward centering loop at {time.time()}")
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
    target_rotation = 300
    rotating_angle = 60
    offset = 0
    
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
            if abs(deg_turned - 60) < 15:
                offset = 0
            offset = deg_turned
            continue
            # continue  # loop back, read visible again WITHOUT rotating
        elif result in (RotationResult.ABORTED, RotationResult.TIMEOUT):
            print(f"[SCAN] Rotation ended with {result} — stopping scan")
            return
        rotation += 60
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

def movement_thread(controller: DroneController, marker_client):
    print("Starting movement thread...")

    uwb_raw = (0,0,0)
    while uwb_raw == (0,0,0):
        uwb_raw = get_target_position(tag_id)
    uwb_pos = [uwb_raw[0]*100, uwb_raw[1]*100]
    uwb_ground_height = uwb_raw[2]*100
    print(f" Ground pos: {uwb_pos}")
    pos = uwb_pos
    
    controller.start_pose = uwb_pos

    # marker_client.client_takeoff_simul([99], f'Battery: {controller.drone.get_battery()}')
    marker_client.client_takeoff_simul([99], f'Battery: {controller.drone.get_battery()} - Start Pose: {controller.start_pose}')
    print("Taking off...")
    controller.drone.takeoff()
    controller.has_taken_off = True
    controller.drone.move_up(30)
    time.sleep(1)
    
    execute_waypoints(controller, marker_client=marker_client)

def validate_waypoints():
    global start_wpt

    if pi_id in group_1:
        with open('waypoint6.json', 'r') as f:
            data = json.load(f)

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

def execute_waypoints(controller: DroneController, marker_client: MarkerClient):
    waypoint_id = 0
    try:

        if pi_id in group_1:
            with open('waypoint6.json', 'r') as f:
                data = json.load(f)
        else:
            with open('waypoints_uwb.json', 'r') as f:
                data = json.load(f) 
        
        # Execute each waypoint
        for wp in data['wp']:
            # Handle rotation
            status = "Orienting"
            # scan_marker(controller, marker_client=marker_client) #Check the availability of the marker_client
            time.sleep(random.uniform(0, 0.5))
            while marker_client.send_update('waypoint', waypoint_id, detected=True) is False:
                print(f"waiting for waypoint {waypoint_id} to be available")
                controller.drone.send_rc_control(0, 0, 0, 0)
                time.sleep(0.5)
            
            if waypoint_id != 0:
                print(f"Release the waypoints: {waypoint_id - 1}")
                marker_client.send_update('waypoint', marker_id=waypoint_id-1, detected=False)

            # marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
            # time.sleep(0.2)

            # while marker_client.is_waypoint_available(waypoint_id) is False:
            #     print(f"waiting for waypoint {waypoint_id} to be available")
            #     controller.drone.send_rc_control(0, 0, 0, 0)
            #     time.sleep(1)

            # marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)

            status = "Proceeding forward"
            try:
                target_xy = (
                wp['position_cm']['x'],  # x increment
                wp['position_cm']['y'],  # y increment
                )
                print(f"The target position: {target_xy}")
                rc_move_to(controller, target_xy_cm=target_xy)
            except:
                waypoint_id -= 1
                continue
                
            print(f"Drone at Waypoint {waypoint_id}")
            controller.set_current_waypoint(waypoint=waypoint_id)
            marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
            # if waypoint_id > 3:
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
    controller = DroneController(pi_id=pi_id, tag_id=tag_id, network_config=network_config)
    markerclient = MarkerClient(drone_id=controller.drone_id)
    try:
        uwb_thread = threading.Thread(target=uwb_poll_thread, args=(controller.drone_uwbtag, controller), daemon=True)
        video_handler = threading.Thread(target=video_thread, args=(controller,), daemon=True)
        move_handler  = threading.Thread(target=movement_thread, args=(controller,markerclient,), daemon=True)

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
    if validate_waypoints():
        print("Validation passed. Starting execution...")
        main()
    else:
        print("Validation failed. Please check warnings above.")