import threading
import sys
from djitellopy import Tello
import cv2
import time
import cv2.aruco as aruco
import numpy as np
import json
from typing import Dict, Set, Any, List, Literal
import math
import os
import re
from FarMarker_DownwardsCenteringv2 import brake
from UWB_Manipulation.UWB_Reader import get_target_position
from swarmserver.swarmserverclientnew_demo import MarkerClient
from shared_utils.shared_utils import *
from shared_utils.customtello import CustomTello

params = load_params()
tag_id = params.UWBTAG_ID  
delay = params.TAKEOFF_DELAY
drone_id = params.PI_ID

# havent add my danger offset code
np.set_printoptions(legacy='1.25')

# Danger marker parameters
DANGER_DISTANCE_THRESHOLD = 160  # cm
MAX_OFFSET_DISTANCE = 70  # Maximum offset in cm
DANGER_SCALING_FACTOR = 68  # Scaling factor for offset calculation
LOCAL_SEARCH_ASCENDING_SPEED = 20 #Ascending speed during local search in cm/s

# Marker classification
victim_markers = list(range(1, 9))
fire_markers = list(range(11, 15))  # Markers 11-14 are danger markers
relay_markers = list(range(16, 20))

center_tolerance = 8
kx = 0.6
ky = 0.6
descend_speed = 20
min_height = 60

script_name = os.path.basename(__file__)

aruco_valid_ids = set(range(1, 9))
invalid_ids = set(range(11, 15))
group_1 = [5]
group_2 = []
group_3 = []
group_4 = []

id = 0
dis = [1 for i in range(50)]
ang = [1 for i in range(50)]
#pos = [0 for i in range(2)] # Position of drone [x, y]
marker_located = [0 for i in range(50)] # 0 for not located, 1 for located
marker_located[0] = 1
marker_list = []
status = ""
course = 0

LAND_ID = 0 # set to 0 for no land
FLYING_STATE = False

#Researching Mode
fire_searching_mode = False

## ADDED FROM SECOND CODE - Downward centering parameters
DOWN_CENTERING_THRESHOLD = 10  # Reduced from 15 to match working code
RC_SPEED_SCALE = 0.3
CONSECUTIVE_FRAMES_REQUIRED = 15

waypoints = [] # to store executed waypoints and drone's current position

#cam_mat = params.CAMERA_MATRIX
#dist_coef = params.DIST_COEFF

MARKER_SIZE = 19 # centimeters (measure your printed marker size)
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
marker_dict_50 = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)  # Added for downward camera
param_markers = aruco.DetectorParameters()
stream_ready = threading.Event()

# ADDED FROM SECOND CODE - Functions for downward centering

def get_calibration_params(controller):
    K = np.array([
        [473.11891765, 0.000000, 323.13378419],
        [0.000000, 475.40414375, 234.45123335],
        [0.000000, 0.000000, 1.000000]
    ])
    D = np.array([0.04172334, 0.38719427, -0.00921904, 0.00348033, -0.70063595])

    if controller.using_down_vision:
        K = np.array([
            [232.08608036, 0.000000, 152.2358733], 
            [0.000000, 232.64995134, 124.98737218], 
            [0.000000, 0.000000, 1.000000]
        ])
    
        D = np.array([5.26408126e-01, -1.52051597e+00, 1.20937906e-02, -1.34771857e-03, 1.29077550e+00])
    return K, D

# NEWLY ADDED - Functions for danger marker handling
# Store last successful offset values
last_successful_offset_x = 0
last_successful_offset_z = 0
offset_timeout = 3.0  # Default timeout in seconds

def detect_marker_pose(gray_frame, drone,
                       aruco_dict_type=cv2.aruco.DICT_5X5_250,
                       marker_size_m=19):
    K, D = get_calibration_params(drone)
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
        if m in aruco_valid_ids and m not in invalid_ids:
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

def perform_drone_alignment(drone, target_marker_id):
    #first_encounter = True
    """Perform precision landing on a specific marker using downward camera"""
    global status, marker_list
    marker_client.send_update('marker', marker_id=int(target_marker_id), detected=True)
    marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True)
    
    for marker_id in marker_list:
        if marker_id != target_marker_id:
            marker_client.send_update('marker', marker_id=int(marker_id), detected=False)

    status = f"Initiating precision alignment on relay marker {target_marker_id}"
    print(status)
    
    # Switch to downward camera
    drone.using_down_vision = True
    time.sleep(5)  # Wait for camera to stabilize
    
    # Get frame reader for downward camera
    time.sleep(1)
    marker_client.send_update('marker', marker_id=int(target_marker_id), detected=True)
    marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True)
    
    count = 0
    # Landing loop
    while drone.is_running:
        marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True)
        
        # Get frame with retry mechanism
        retry_count = 0
        frame = None
        while frame is None and retry_count < 3:
            frame = drone.get_frame()
            if frame is None:
                print("Frame capture failed, retrying...")
                time.sleep(0.1)
                retry_count += 1
        
        if frame is None:
            print("Failed to capture frame after retries")
            drone.send_rc_control(0, 0, 0, 0)
            continue
            
        # Process frame for marker detection
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, _, rvecs, tvecs = detect_marker_pose(gray_frame, drone)

        if rvecs is not None and tvecs is not None:
            # first_encounter = False
            x_offset, y_offset = tvecs[0][0][0], tvecs[0][0][1]

            if abs(x_offset) < center_tolerance and abs(y_offset) < center_tolerance:
                print("Landing.")
                break
            else:
                delta_x = x_offset*kx
                delta_y = y_offset*ky

                lr = int(-delta_y)
                fb = int(-delta_x)

                print(f"Offset x in cm = {x_offset}, offset_y in cm = {y_offset} → "
                  f"lr={lr:.0f}, fb={fb:.0f}")
                    
                drone.send_rc_control(lr, fb, 0, 0)
                time.sleep(0.2)
                drone.send_rc_control(0,0,0,0)
                continue
        else: 
            drone.send_rc_control(0,0,10,0)
            time.sleep(0.3)
            if drone.get_height() > 130: 
                break
            # count =+ 1
            # if count > 3:
            #     drone.send_rc_control(0,0,0,0)
            #     print("Unsuccessfully landed on marker")
            #     break
            # else: 
            #     drone.send_rc_control(0,0,10,0)
            drone.send_rc_control(0,0,0,0)
            time.sleep(1)
    drone.using_down_vision = False

def perform_precision_landing(drone, target_marker_id):
    """Perform precision landing on a specific marker using downward camera"""
    global status, marker_list
    def rc_pulse(lr=0, fb=0, ud=0, dur=0.3):
        drone.send_rc_control(int(lr), int(fb), int(ud), 0)
        time.sleep(float(dur))
        drone.send_rc_control(0, 0, 0, 0)
    marker_client.send_update('marker', marker_id=int(target_marker_id), detected=True)
    marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True)
    
    for marker_id in marker_list:
        if marker_id != target_marker_id:
            marker_client.send_update('marker', marker_id=int(marker_id), detected=False)

    status = f"Initiating precision landing on marker {target_marker_id}"
    print(status)
    
    # Switch to downward camera
    drone.using_down_vision = True
    time.sleep(5)  # Wait for camera to stabilize
    
    # Get frame reader for downward camera
    time.sleep(1)
    marker_client.send_update('marker', marker_id=int(target_marker_id), detected=True)
    marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True)
    
    count = 0
    # Landing loop
    while drone.is_running:
        marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True)
        
        # Get frame with retry mechanism
        retry_count = 0
        frame = None
        while frame is None and retry_count < 3:
            frame = drone.get_frame()
            if frame is None:
                print("Frame capture failed, retrying...")
                time.sleep(0.1)
                retry_count += 1
        
        if frame is None:
            print("Failed to capture frame after retries")
            drone.send_rc_control(0, 0, 0, 0)
            continue
            
        # Process frame for marker detection
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        corners, _, rvecs, tvecs = detect_marker_pose(gray_frame, drone)

        if rvecs is not None and tvecs is not None:
            x_offset, y_offset = tvecs[0][0][0], tvecs[0][0][1]

            if abs(x_offset) < center_tolerance and abs(y_offset) < center_tolerance:
                print("Landing.")
                break
            else:
                delta_x = x_offset*kx
                delta_y = y_offset*ky

                lr = int(-delta_y)
                fb = int(-delta_x)

                print(f"Offset x in cm = {x_offset}, offset_y in cm = {y_offset} → "
                  f"lr={lr:.0f}, fb={fb:.0f}")
                    
                drone.send_rc_control(lr, fb, 0, 0)
                time.sleep(0.2)
                drone.send_rc_control(0,0,0,0)
                continue
        else: 
            drone.send_rc_control(0,0,10,0)
            time.sleep(0.3)
            if drone.get_height() > 130: 
                break
            # count =+ 1
            # if count > 3:
            #     drone.send_rc_control(0,0,0,0)
            #     print("Unsuccessfully landed on marker")
            #     break
            # else: 
            #     drone.send_rc_control(0,0,10,0)
            drone.send_rc_control(0,0,0,0)
            time.sleep(1)
    
    print("Centered — descending slightly...")
    rc_pulse(0, 0, descend_speed, 0)
    time.sleep(0.3)

    h = drone.get_height()
    if h is not None and h <= min_height:
        print(f"Near ground (h={h} cm) — landing.")

    drone.send_rc_control(0, 0, 0, 0)
    drone.land()
    print("Landed safely.")
    drone.is_running = False

###########################################################################################################
def scan_marker(drone, release = False):
    """This function checks if the marker is registered in the marker list of the server or not"""
    global marker_client

    #Check if that marker is available, if yes, we would put the dis[id] equals to the distance from drone to the marker
    for id in marker_list:
        if marker_client.is_marker_available(id) and id in victim_markers:
            print(f"Marker ID {id} is being scanned. ")
            
            bonus_detected = False
            if int(id) == 2:
                print("Bonus victim is detected!")
                bonus_detected = True
            marker_client.send_update('marker', marker_id=int(id), detected=True, bonus_detected=bonus_detected)
            
            # if release == True:
            #     marker_client.send_update('waypoint', marker_id=waypoint_id-1, detected=False)
            locate_marker(drone,id)

#Spin around and scan for marker
def scan_for_marker(drone):
    """This function commands the drone to spin 360 degree while also searching for marker every 60 degrees"""
    global ang, heading, dis, marker_IDs, status, course, marker_client
    rotation = 0
    while rotation < 360:
        for id in marker_list:
                #if marker_located[id] == 0:
                bonus_detected = False
                if marker_client.is_marker_available(id) and id in victim_markers:
                    if int(id) not in fire_markers and fire_searching_mode:
                        continue
                    if int(id) == 2:
                        print("Bonus victim is detected!")
                        bonus_detected = True
                    marker_client.send_update('marker', marker_id=int(id), detected=True, bonus_detected=bonus_detected)
                    drone.send_rc_control(0, 0, 0, 0)
                    print(f"Measuring Marker {id}'s position...")
                    status = f"Measuring Marker {id}'s position..."
                    locate_marker(drone,id)
        drone.rotate_clockwise(60)
        rotation += 60
        time.sleep(1)
        

    if abs(heading-course) <= 180:
        drone.rotate_counter_clockwise(heading-course)
    elif heading-course < -180:
        drone.rotate_counter_clockwise(360 + heading-course)
    else:
        drone.rotate_counter_clockwise(heading-course - 360)

def locate_marker(drone, id):
    fire_searching = True
    if int(id) in fire_markers:
        fire_searching = False

    """Calculate the distance/heading from drone to the specific marker"""
    while True:
        global marker_client, ang, heading, dis, status, marker_located

        marker_client.send_update('marker', marker_id=int(id), detected=True)

        #Reset distance and angle for this marker id
        dis[id] = 0
        time.sleep(1)
        #ang[id] = 0 
        #time.sleep(0.5)
        p_const = 0.1

        cycle = 0
        status = f"Aligning with marker {id}"
        # while abs(ang[id]) > 1:  # Aligning with marker #id
        #     marker_client.send_update('marker', marker_id=int(id), detected=True)
        #     if cycle < 2:
        #         drone.rotate_clockwise(int(ang[id]*1.2))
        #     else:
        #         drone.rotate_clockwise(int(ang[id]*1.8))
        #     cycle += 1
        #     ang[id] = 0  # Reset angle for each cycle
        #     time.sleep(1)
        #     if cycle == 7:
        #         break

        #Nguyen Tuan Minh Feb 4 - Add this for marker alignment
        while abs(ang[id]) > 5:  # Aligning with marker #id
            rotating_angle = ang[id]
            if rotating_angle > 1000:
                rotating_angle = 0
            if abs(rotating_angle) <= 50:
                p_const = 0.4
            print(f"Current Azimuth Angle: {rotating_angle}")
            marker_client.send_update('marker', marker_id=int(id), detected=True)
            drone.rotate_clockwise(int(rotating_angle*p_const))
            cycle += 1
            time.sleep(1)
            #ang[id] = 0
            if cycle == 15:
                break
        print(f"Azimuth angle after adjustment: {ang[id]}")
        #Why reset the distance to 0 first? Is this variable supposed to be updated continuously with the video thread?
        dis[id] = 0
        print(f"Calculating Marker {id}'s position...")
        status = f"Calculating Marker {id}'s position..."
        marker_client.send_update('marker', marker_id=int(id), detected=True)
        time.sleep(3) #Wait for 3 seconds to be updated
        
        if dis[id] == 0:
            print(f"Marker {id} was lost")
            status = f"Marker {id} was lost"

            marker_located[id] = 0
            marker_client.send_update('marker', marker_id=int(id), detected=False)
            if id in marker_list:
                marker_list.pop(marker_list.index(id))
                break
        else:
            # Now move forward to be close to the marker
            print(f"Value of the distance: {dis[id]}")
            distance_forward = dis[id]

            total_forward_distance = distance_forward*1.5
            num_segments = 4
            forward_per_segment = int(total_forward_distance / num_segments)
            time.sleep(6)
            # marker_client.send_update('waypoint', marker_id=waypoint_id, detected=False)

            try:
                current_altitude = drone.get_height()
            except:
                current_altitude = 100  # Default to 100 cm if retrieval fails
            print(f"Current altitude: {current_altitude} cm")
            target_altitude = 30

            total_descent = max(current_altitude - target_altitude, 0)
            descent_per_segment = int(total_descent / num_segments)
            initial_seg = forward_per_segment
            for i in range(num_segments):
                print(f"\nExecuting segment {i+1}/{num_segments}")
                
                # if i != 0:
                #     # Nguyen Tuan Minh Feb 4 - Adjustment for yaw at each points on the segments
                #     # while abs(ang[id]) > 10:  # Aligning with marker #id
                #     #     marker_client.send_update('marker', marker_id=int(id), detected=True)
                #     #     drone.send_rc_control(0,0,0, ang[id]*p_const)
                #     #     time.sleep(0.5)
                #     #     drone.send_rc_control(0,0,0,0)
                #     #     time.sleep(0.5)
                #     forward_per_segment = dis[id]/(num_segments-i)
                #     print("This line does work bitch")
                # else:
                #     forward_per_segment = initial_seg
                drone.move_forward(int(forward_per_segment))
                time.sleep(1)
                brake(drone, 300)
                time.sleep(0.6)
        
                descent_velocity = descent_per_segment/0.5
                drone.send_rc_control(0, 0, -int(descent_velocity), 0)  # gentle down velocity (cm/s)
                time.sleep(0.5)
                drone.send_rc_control(0, 0, 0, 0)

                brake(drone, 300)
                time.sleep(0.5)
            time.sleep(0.4)
            offset_applied = False

            # Different handling based on whether a danger marker was detected
            if offset_applied:
                print("Nah it won't")
            else:
                # Scenario 2: No danger marker - use precision landing with downward camera
                status = f"Beginning precision landing on marker {id}"
                print(status)
                
                # Perform precision landing
                marker_client.send_update('marker', marker_id=int(id), detected=True)
                # landing_success = perform_precision_landing(drone, id)
                
                # if landing_success:
                #     drone.streamoff()
                #     drone.reboot()
                #     marker_client.send_update('marker', marker_id=int(id), landed=True)
                #     status = f"Successfully landed on marker {id}"
                #     drone.is_running = False
                # else:
                #     status = f"Failed precision landing on marker {id}"
                #     # Fall back to regular landing if precision landing fails
                #     drone.land()
                #     drone.streamoff()
                #     drone.reboot()
                #     drone.is_running = False
                #     marker_client.send_update('marker', marker_id=int(id), landed=True)
                if int(id) in relay_markers:
                    perform_drone_alignment(drone, id)
                    bonus_pose = get_target_position(target_id=tag_id)
                    marker_client.send_update('marker', marker_id=int(id), detected=True, bonus_detected=True, bonus_position=bonus_pose)
                    
                if fire_searching: 
                    fire_searching_mode = True
                    scan_for_marker(drone)
    
                perform_precision_landing(drone, id)
                drone.streamoff()
                drone.reboot()
                marker_client.send_update('marker', marker_id=int(id), landed=True)
                drone.is_running = False
                #drone.land()
            break

def ascend(drone, altitude):
    global height, status

    if abs(height-altitude)>10:
        drone.go_xyz_speed(0,0,int(altitude-height),100)
        print(f"Ascending to height {altitude}")
        status = f"Ascending to height {altitude}"
    else:
        print(f"Already at {height}")
        status = f"Already at {height}"

def stream_video(drone):
    global heading, pos, ang, height, marker_IDs, marker_list, status, dis, id, sys, course, switch
    frame_reader = drone.get_frame_read()
    switch = True
    while True:
        if drone.using_down_vision and switch:
            retry = 0
            max_retries = 3
            while retry < max_retries:
                drone.send_command_with_return("downvision 1")
                time.sleep(1)
                retry += 1
            time.sleep(5)
            switch = False
        
        #ret = True
        frame = frame_reader.frame
        #frame = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)

        height = drone.get_height()
        battery = drone.get_battery()
        heading = drone.get_yaw() - start_heading

        gray_frame = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
        marker_corners, marker_IDs, reject = cv2.aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )
        
        cv2.putText(frame,f"Height: {height} cm | Heading: {heading} degrees | Course: {course} degrees | Battery: {battery} %",(10,20),cv2.FONT_HERSHEY_PLAIN,0.7,(0,255,255),2)
        cv2.putText(frame,f"Position: X: {round(pos[0], 2)} Y: {round(pos[1], 2)} | Status: {status}",(10,40),cv2.FONT_HERSHEY_PLAIN,0.7,(0,255,255),2)
        
        cam_mat, dist_coef = get_calibration_params(drone)
        
        if marker_corners:
            rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                marker_corners, MARKER_SIZE, cam_mat, dist_coef
            )
            total_markers = range(0, marker_IDs.size)
            
            # Track markers of interest
            visible_markers = []
            visible_dangers = []
            
            for ids, corners, i in zip(marker_IDs, marker_corners, total_markers):
                if ids[0] not in marker_list and ids[0] != 0 and ids[0] < 20:
                    marker_list.append(ids[0])
                    print(f"New marker {ids[0]} is detected")
                
                if ids[0] > 20:
                    continue
                
                # Highlight danger markers in red
                if ids[0] in fire_markers:
                    marker_color = (0, 0, 255)  # Red for danger markers
                    visible_dangers.append(ids[0])
                else:
                    marker_color = (0, 255, 255)  # Yellow for regular markers
                    visible_markers.append(ids[0])
                
                cv2.polylines(
                    frame, [corners.astype(np.int32)], True, marker_color, 4, cv2.LINE_AA
                )

                # Calculating the distance
                
                x_cm = float(tVec[0][0][0])
                y_cm = float(tVec[0][0][1])
                z_cm = float(tVec[0][0][2])

                #horiz_cm = (x_cm**2 + z_cm**2) ** 0.5
                horiz_cm = z_cm
                dis[ids[0]] = horiz_cm
                #angle = 10/(30+(horiz_cm-150)/5)*(round(tVec[i][0][0],1))
                #angle = math.atan2(x_cm, z_cm)
                ang[ids[0]] = x_cm

                # Draw the pose of the marker
                cv2.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 0.05)

                mc = np.mean(corners[0], axis=0).astype(int)
                cv2.circle(frame, (mc[0], mc[1]), 5, (0, 0, 255), -1)

                cv2.putText(frame, f"X:{x_cm:+.1f} Y:{y_cm:+.1f} Z(fwd):{z_cm:.1f} H:{horiz_cm:.1f}",
                    (10, 30 + 22*i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,0), 2)
            
            # Display danger marker information if any are visible
            if visible_dangers:
                cv2.putText(
                    frame,
                    f"Danger markers: {visible_dangers}",
                    (10,60),
                    cv2.FONT_HERSHEY_PLAIN,
                    0.7,
                    (0,0,255),
                    2
                )

        # Check if streaming readiness hasn't been signaled yet
        if not stream_ready.is_set():
            # Signal that video streaming is ready
            stream_ready.set()
            print("Event Signal Set: Stream is live.")
        
        if cv2.waitKey(1) & 0xFF == ord('z'):
            break
        
        drone.set_frame(frame)

    #cap.release()
    cv2.destroyAllWindows()
    sys.exit()

###########################################################################################################

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
    
    global pos, heading
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


def validate_waypoints():

    global start_wpt

    if drone_id in group_1:
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

def execute_waypoints(drone):
    global LAND_ID, FLYING_STATE, waypoints, status, course, pos, waypoint_id
    
    waypoint_id = 0 # Waypoint 0 is the starting waypoint
    while marker_client.is_waypoint_available(waypoint_id) is False:
        print(f"waiting for waypoint {waypoint_id} to be available")
        drone.send_rc_control(0, 0, 0, 0)
        time.sleep(1)
    marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
    print(int(start_wpt[1]-sy),int(sx-start_wpt[0]))
    if abs(int(start_wpt[1]-sy)) > 20 or  abs(int(sx-start_wpt[0])) > 20:
        if abs(int(start_wpt[1]-sy)) > 1500 or  abs(int(sx-start_wpt[0])) > 1500:
            print("More than 1500cm")
            drone.go_xyz_speed(int((start_wpt[1]-sy)/4), int((sx-start_wpt[0])/4), 0, 100)
            pos[0] += (start_wpt[0] - sx)/4
            pos[1] += (start_wpt[1] - sy)/4
            uwb_correction(drone)
            drone.go_xyz_speed(int((start_wpt[1]-sy)/4), int((sx-start_wpt[0])/4), 0, 100)
            pos[0] += (start_wpt[0] - sx)/4
            pos[1] += (start_wpt[1] - sy)/4
            uwb_correction(drone)
            drone.go_xyz_speed(int((start_wpt[1]-sy)/4), int((sx-start_wpt[0])/4), 0, 100)
            pos[0] += (start_wpt[0] - sx)/4
            pos[1] += (start_wpt[1] - sy)/4
            uwb_correction(drone)
            drone.go_xyz_speed(int((start_wpt[1]-sy)/4), int((sx-start_wpt[0])/4), 0, 100)
            pos[0] += (start_wpt[0] - sx)/4
            pos[1] += (start_wpt[1] - sy)/4
        elif abs(int(start_wpt[1]-sy)) > 1000 or  abs(int(sx-start_wpt[0])) > 1000:
            print("More than 1000cm")
            drone.go_xyz_speed(int((start_wpt[1]-sy)/3), int((sx-start_wpt[0])/3), 0, 100)
            pos[0] += (start_wpt[0] - sx)/3
            pos[1] += (start_wpt[1] - sy)/3
            uwb_correction(drone)
            drone.go_xyz_speed(int((start_wpt[1]-sy)/3), int((sx-start_wpt[0])/3), 0, 100)
            pos[0] += (start_wpt[0] - sx)/3
            pos[1] += (start_wpt[1] - sy)/3
            uwb_correction(drone)
            drone.go_xyz_speed(int((start_wpt[1]-sy)/3), int((sx-start_wpt[0])/3), 0, 100)
            pos[0] += (start_wpt[0] - sx)/3
            pos[1] += (start_wpt[1] - sy)/3
        elif abs(int(start_wpt[1]-sy)) > 500 or  abs(int(sx-start_wpt[0])) > 500:
            print("More than 500cm")
            drone.go_xyz_speed(int((start_wpt[1]-sy)/2), int((sx-start_wpt[0])/2), 0, 100)
            pos[0] += (start_wpt[0] - sx)/2
            pos[1] += (start_wpt[1] - sy)/2
            uwb_correction(drone)
            drone.go_xyz_speed(int((start_wpt[1]-sy)/2), int((sx-start_wpt[0])/2), 0, 100)
            pos[0] += (start_wpt[0] - sx)/2
            pos[1] += (start_wpt[1] - sy)/2
        elif abs(int(start_wpt[1]-sy)) <= 500 and  abs(int(sx-start_wpt[0])) <= 500:
            print("Less than 500cm")
            drone.go_xyz_speed(int(start_wpt[1]-sy), int(sx-start_wpt[0]), 0, 100)
            pos[0] += start_wpt[0] - sx
            pos[1] += start_wpt[1] - sy
        print(f"Starting position: {pos}")
    else:
        print("Already at starting waypoint")
    # marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
    time.sleep(3)
    uwb_correction(drone)
    # marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
    if abs(heading) > 1:
        drone.rotate_counter_clockwise(int(heading))

    try:

        # Initialize position and orientation
        abs_position = {"x": start_wpt[0], "y": start_wpt[1]}  # in cm
        orientation = 180  # Starting heading in degrees (assuming 180 as the initial heading)
        
        if drone_id in group_1:
            with open('waypoint6.json', 'r') as f:
                data = json.load(f)
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
                while abs(course - heading) > 3:
                    if -180 < course - heading < 180:
                        drone.send_command_without_return(f"cw {int(course - heading)}")
                    elif course - heading > 180:
                        drone.send_command_without_return(f"cw {int(course - heading) - 360}")
                    else:
                        drone.send_command_without_return(f"cw {int(course - heading) + 360}")
                    time.sleep(4)
                
                scan_marker(drone)

                # marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
                update_position(waypoints, abs_position, orientation)
                uwb_correction(drone)

            # Check if drone is facing course direction before flying forward
            while abs(course - heading) > 5:
                if -180 < course - heading < 180:
                    drone.send_command_without_return(f"cw {int(course - heading)}")
                elif course - heading > 180:
                    drone.send_command_without_return(f"cw {int(course - heading) - 360}")
                else:
                    drone.send_command_without_return(f"cw {int(course - heading) + 360}")
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
                        drone.send_rc_control(0, 0, 0, 0)
                        # marker_client.send_update('waypoint', marker_id=waypoint_id-1, detected=True)
                        time.sleep(1)

                    marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
                    try:
                        drone.move_forward(200)
                    except:
                        waypoint_id -= 1
                        continue
                    
                    print(f"Drone at to Waypoint {waypoint_id}")
                    scan_marker(drone, True)
                    update_position(waypoints, abs_position, orientation, 200)
                    time.sleep(2)
                    uwb_correction(drone)
                    time.sleep(2)
                    # marker_client.send_update('waypoint', marker_id=waypoint_id-1, detected=False)
                    scan_for_marker(drone)
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
                    drone.move_forward(distance)
                except:
                    waypoint_id -= 1
                    continue
                
                print(f"Drone at to Waypoint {waypoint_id}")
                scan_marker(drone, True)
                update_position(waypoints, abs_position, orientation, distance)
                time.sleep(3)
                uwb_correction(drone)
                
                marker_client.send_update('waypoint', marker_id=waypoint_id-1, detected=False)
                scan_for_marker(drone)
                time.sleep(1)
    
    except Exception as e:
        print(f"Error occurred: {e}")
    
    finally:
        if drone.is_flying:
            print("Landing...")
            status = "Landing..."
            drone.land()
            drone.streamoff()
        print("Mission completed!")

def update_position(waypoints, position, orientation, distance=0):
    global pos

    rad = math.radians(orientation)
    pos[1] -= int(distance * math.cos(rad))
    pos[0] += int(distance * math.sin(rad))
    position["x"] += int(distance * math.cos(rad))
    position["y"] += int(distance * math.sin(rad))
    waypoints.append({"x": position["x"], "y": position["y"], "orientation": orientation, "distance": distance})

###########################################################################################################

def flight_routine(drone):

    print("waiting for event to signal video stream readiness")
    stream_ready.wait()
    print("event signaled for video stream readiness")

    ascend(drone,90)
    drone.rotate_counter_clockwise(heading)
    time.sleep(2)
    
    execute_waypoints(drone)

def main():
    global marker_client, pos, sx, sy, uwb_ground_height, start_heading

    # Initialize the drone, connect to it, and turn its video stream on.
    drone = CustomTello(network_config=params.NETWORK_CONFIG)

    #print(f"[DEBUG] Connecting to {getattr(drone, 'host', getattr(drone, '_host', '??'))}:{getattr(drone, 'port', getattr(drone, '_port', '??'))}")
    drone.connect()
    drone.is_running = True
    marker_client = MarkerClient(drone_id)
    uwb_raw = (0,0,0)
    while uwb_raw == (0,0,0):
        uwb_raw = get_target_position(tag_id)
    uwb_pos = [uwb_raw[0]*100, uwb_raw[1]*100]
    uwb_ground_height = uwb_raw[2]*100
    print(f" Ground pos: {uwb_pos}")
    pos = uwb_pos
    sx, sy = uwb_pos
    start_heading = drone.get_yaw()
    marker_client.client_takeoff_simul([99], f'Battery: {drone.get_battery()} Pos: {int(pos[0]), int(pos[1])}')

    #time.sleep(180)
    drone.takeoff()

    delay_count = 0
    while delay_count < delay:
        drone.send_rc_control(0, 0, 0, 0)
        print(drone.get_battery())
        time.sleep(5)
        delay_count += 5
        print(delay_count)

    drone.streamon()
    print("drone connected and stream on. Starting video stream thread.\n")
    stream_thread = threading.Thread(target=stream_video, args=(drone,))
    stream_thread.daemon = True
    stream_thread.start()
    drone.send_command_with_return("downvision 0")

    # Execute the flight routine
    flight_routine(drone)

    print("Flight routine ended.")

    # Reboot the drone at the end
    #drone.reboot()

if __name__ == "__main__":
    print("Validating waypoints...")
    if validate_waypoints():
        print("Validation passed. Starting execution...")
        main()
    else:
        print("Validation failed. Please check warnings above.")