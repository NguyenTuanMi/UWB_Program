import threading
import socket
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

# Marker classification
victim_markers = list(range(1, 9))
danger_markers = list(range(11, 15))  # Markers 11-14 are danger markers

script_name = os.path.basename(__file__)


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

## ADDED FROM SECOND CODE - Downward centering parameters
DOWN_CENTERING_THRESHOLD = 10  # Reduced from 15 to match working code
RC_SPEED_SCALE = 0.3
CONSECUTIVE_FRAMES_REQUIRED = 15

waypoints = [] # to store executed waypoints and drone's current position

cam_mat = params.CAMERA_MATRIX
dist_coef = params.DIST_COEFF

MARKER_SIZE = 19 # centimeters (measure your printed marker size)
marker_dict = aruco.getPredefinedDictionary(aruco.DICT_5X5_250)
marker_dict_50 = aruco.getPredefinedDictionary(aruco.DICT_5X5_50)  # Added for downward camera
param_markers = aruco.DetectorParameters()
stream_ready = threading.Event()

###########################################################################################################






###########################################################################################################

# ADDED FROM SECOND CODE - Functions for downward centering
def calculate_rc_values(marker_center, frame_width, frame_height):
    """Calculate RC control values based on marker position"""
    frame_center = (frame_width // 2, frame_height // 2)
    dx = marker_center[0] - frame_center[0]
    dy = marker_center[1] - frame_center[1]
    
    # Scale the RC values based on pixel distance
    left_right = int(dx * RC_SPEED_SCALE)
    forward_back = int(-dy * RC_SPEED_SCALE)
    
    # Clamp values between -100 and 100
    left_right = max(min(left_right, 100), -100)
    forward_back = max(min(forward_back, 100), -100)
    
    return left_right, forward_back

def check_if_centered(marker_center, frame_width, frame_height):
    """Check if marker is centered within threshold"""
    frame_center = (frame_width // 2, frame_height // 2)
    dx = marker_center[0] - frame_center[0]
    dy = marker_center[1] - frame_center[1]
    return abs(dx) <= DOWN_CENTERING_THRESHOLD and abs(dy) <= DOWN_CENTERING_THRESHOLD

def process_frame(frame):
    """Process frame for marker detection with proper orientation handling"""
    if frame is None or frame.size == 0:
        return None, None, None, frame
        
    # Make a deep copy to avoid reference issues
    frame = np.array(frame, copy=True)
    
    # Always ensure frame is in the correct orientation
    if frame.shape[0] < frame.shape[1]:  # if height < width
        frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        
    # Detect markers
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, marker_dict_50)
    
    if ids is not None and len(ids) > 0:
        marker_center = np.mean(corners[0][0], axis=0)
        return marker_center, corners[0], ids[0], frame
    return None, None, None, frame

# NEWLY ADDED - Functions for danger marker handling
# Store last successful offset values
last_successful_offset_x = 0
last_successful_offset_z = 0
offset_timeout = 3.0  # Default timeout in seconds

def calculate_marker_distance(valid_position, danger_position):
    """
    Calculate the ground distance between a valid marker and a danger marker
    
    Parameters:
    valid_position (tuple): (x, y, z) position of valid marker
    danger_position (tuple): (x, y, z) position of danger marker
    
    Returns:
    float: Ground distance between markers in cm
    """
    # Extract x and z coordinates (ground plane)
    valid_x, _, valid_z = valid_position
    danger_x, _, danger_z = danger_position
    
    # Calculate Euclidean distance on the ground plane (x-z plane)
    ground_distance = np.sqrt((valid_x - danger_x)**2 + (valid_z - danger_z)**2)
    
    return ground_distance

def prepare_offset(valid_position, danger_position):
    """
    Calculate offset to avoid danger marker while staying close to valid marker
    
    Parameters:
    valid_position (tuple): (x, y, z) position of valid marker
    danger_position (tuple): (x, y, z) position of danger marker
    
    Returns:
    tuple: (offset_x, offset_z) - offset to apply in cm
    """
    # Calculate displacement vector from danger to valid marker
    valid_x, _, valid_z = valid_position
    danger_x, _, danger_z = danger_position
    
    # Ground distance between markers
    ground_distance = calculate_marker_distance(valid_position, danger_position)
    
    # Direction vector from danger to valid marker
    dx = valid_x - danger_x
    dz = valid_z - danger_z
    
    # Normalize the direction
    if ground_distance > 0:
        direction_x = dx / ground_distance
        direction_z = dz / ground_distance
    else:
        direction_x, direction_z = 0, 0
    
    # Calculate perpendicular offset vector (90 degrees to the connecting line)
    perpendicular_x = -direction_z
    perpendicular_z = direction_x
    
    # Scale the offset
    offset_distance = -68  # cm to move
    offset_x = perpendicular_x * offset_distance
    offset_z = perpendicular_z * offset_distance
    
    print(f"Pre-calculated offset: x={offset_x:.1f} cm, z={offset_z:.1f} cm")
    print(f"Ground distance between markers: {ground_distance:.1f} cm")
    
    return int(offset_x), int(offset_z)

def check_and_apply_danger_offset_with_timeout(drone, target_id, marker_corners, marker_IDs, rVec, tVec, timeout_seconds=3.0, calculate_only=False):
    """
    Check if any danger markers are too close to the target marker and calculate offset with timeout
    
    Parameters:
    drone: Tello drone object
    target_id: ID of the target marker
    marker_corners: corners of detected markers
    marker_IDs: IDs of detected markers
    rVec, tVec: pose estimation vectors
    timeout_seconds: maximum time to wait for offset calculation
    calculate_only: if True, only calculate the offset but don't apply it
    
    Returns:
    tuple: (bool, offset_x, offset_z) - whether offset was needed and the offset values
    """
    global last_successful_offset_x, last_successful_offset_z
    
    start_time = time.time()
    calculated_offset = False
    offset_x, offset_z = 0, 0
    
    while time.time() - start_time < timeout_seconds and not calculated_offset:
        if marker_IDs is None or len(marker_IDs) == 0:
            time.sleep(0.1)
            continue
        
        # Find target marker position
        target_position = None
        for i, id_val in enumerate(marker_IDs):
            if id_val[0] == target_id:
                x, y, z = tVec[i][0]
                target_position = (x, y, z)
                break
        
        if target_position is None:
            time.sleep(0.1)
            continue
        
        # Find closest danger marker
        closest_danger_position = None
        closest_danger_distance = float('inf')
        closest_danger_id = None
        
        for i, id_val in enumerate(marker_IDs):
            marker_id = id_val[0]
            if marker_id in danger_markers:
                x, y, z = tVec[i][0]
                danger_position = (x, y, z)
                
                # Calculate ground distance to target
                ground_distance = calculate_marker_distance(target_position, danger_position)
                
                if ground_distance < closest_danger_distance:
                    closest_danger_distance = ground_distance
                    closest_danger_position = danger_position
                    closest_danger_id = marker_id
        
        # If a danger marker is close enough, calculate offset
        if closest_danger_position and closest_danger_distance <= DANGER_DISTANCE_THRESHOLD:
            print(f"Danger marker {closest_danger_id} detected {closest_danger_distance:.1f}cm from target marker {target_id}")
            offset_x, offset_z = prepare_offset(target_position, closest_danger_position)
            calculated_offset = True
            
            # Store successful offset values
            last_successful_offset_x = offset_x
            last_successful_offset_z = offset_z
        else:
            time.sleep(0.1)
    
    # If timeout occurred without finding offset, use last known values
    if not calculated_offset:
        print(f"Offset calculation timed out. Using last known values: x={last_successful_offset_x} cm, z={last_successful_offset_z} cm")
        offset_x = last_successful_offset_x
        offset_z = last_successful_offset_z
        
        # Only consider offset needed if we have valid last values
        if offset_x != 0 or offset_z != 0:
            calculated_offset = True
    
    # Apply the offset if calculated and not in calculate_only mode
    if calculated_offset and not calculate_only:
        print(f"Applying offset: x={offset_x} cm, z={offset_z} cm")
        drone.go_xyz_speed(offset_x, offset_z, 0, 20)
        time.sleep(2)  # Wait for movement to complete
    
    return calculated_offset, offset_x, offset_z

def perform_precision_landing(drone, target_marker_id):
    """Perform precision landing on a specific marker using downward camera"""
    global status, marker_list
    marker_client.send_update('marker', marker_id=int(target_marker_id), detected=True)
    marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True)
    for marker_id in marker_list:
        if marker_id != target_marker_id:
            marker_client.send_update('marker', marker_id=int(marker_id), detected=False)

    status = f"Initiating precision landing on marker {target_marker_id}"
    print(status)
    
    # Switch to downward camera
    #drone.send_command_with_return("downvision 1")
    drone.using_down_vision = True
    time.sleep(5)  # Wait for camera to stabilize
    
    # Get frame reader for downward camera
    #frame_reader = drone.get_frame_read()
    time.sleep(1)
    marker_client.send_update('marker', marker_id=int(target_marker_id), detected=True)
    marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True)
    consecutive_centered_frames = 0
    landing_initiated = False
    
    # Landing loop
    while True:
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
        marker_center, corners, marker_id, processed_frame = process_frame(frame)
        
        if processed_frame is not None:
            frame_height, frame_width = processed_frame.shape[:2]
            
            if marker_center is not None and (marker_id is None or marker_id == target_marker_id):
                # Marker detected, check if centered
                if check_if_centered(marker_center, frame_width, frame_height):
                    consecutive_centered_frames += 1
                    status = f"Centering on marker {marker_id}: {consecutive_centered_frames}/{CONSECUTIVE_FRAMES_REQUIRED}"
                    print(status)
                    
                    # Stop movement when centered
                    drone.send_rc_control(0, 0, 0, 0)
                    
                    # If centered for enough consecutive frames, start controlled descent
                    if consecutive_centered_frames >= CONSECUTIVE_FRAMES_REQUIRED:
                        status = f"Centered on marker {marker_id}! Starting controlled descent..."
                        print(status)

                        drone.land()
                        drone.streamoff()
                        marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True)
                        return True
                else:
                    # Reset counter if not centered
                    consecutive_centered_frames = 0
                    
                    # Calculate RC values to center the marker
                    lr, fb = calculate_rc_values(marker_center, frame_width, frame_height)
                    drone.send_rc_control(lr, fb, 0, 0)
                    status = f"Adjusting position: LR={lr}, FB={fb}"
            else:
                # No marker detected, slowly descend and search
                print(f"Proceeding to local search for marker {target_marker_id}...")

                search_moves = [
                (0, 20, 0, 0.3),   # forward
                (0, -40, 0, 0.3),   # backward
                (0, 20, 0, 0.3),  # forward to original position
                (-20, 0, 0, 0.3),  # left
                (40, 0, 0, 0.3)   # right
                ]
     
                found = False
                for lr, fb, ud, dur in search_moves:
                    drone.rc_pulse(lr, fb, ud, dur=3)
                    time.sleep(0.5)  # short pause to stabilize

                    # check if marker reappears
                    if frame is not None:
                        new_center, corners, marker_id, processed_frame = process_frame(frame)
                        if new_center is not None and (marker_id is None or marker_id == target_marker_id):
                            print("Marker re-acquired! Resuming centering...")
                            found = True
                            break
                
                if not found: 
                    drone.land()
                    drone.streamoff()
                    marker_client.send_update('marker', marker_id=int(target_marker_id), landed=True)
                    status = "No marker detected after local search"
                
        # Check if we should abort (e.g., timeout, battery low)
        # This is a simplified example - you might want more sophisticated conditions
        if drone.get_battery() < 10:
            print("Battery low, aborting precision landing")
            drone.land()
            drone.streamoff()
            return False
                
        time.sleep(0.05)  # Control loop rate

###########################################################################################################
#This function checks if the marker is registered in the marker list of the server or not
def scan_marker(drone, release = False):
    global marker_client

    for id in marker_list:
        if marker_client.is_marker_available(id) and id in victim_markers:
            bonus_detected = False
            if id == 2:
                print("Bonus victim is detected!")
                bonus_detected = True
            marker_client.send_update('marker', marker_id=int(id), detected=True, bonus_detected=bonus_detected)
            # if release == True:
            #     marker_client.send_update('waypoint', marker_id=waypoint_id-1, detected=False)
            locate_marker(drone,id)

#Spin around and scan for marker
def scan_for_marker(drone):
    global ang, heading, dis, marker_IDs, status, course, marker_client
    rotation = 0
    while rotation < 360:
        for id in marker_list:
                #if marker_located[id] == 0:
                if marker_client.is_marker_available(id) and id in victim_markers:
                    marker_client.send_update('marker', marker_id=int(id), detected=True)
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
    # marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)

'''
def scan_for_marker(drone):
    global ang, heading, dis, marker_IDs, status, course, marker_client
    rotation = 0
    while rotation < 360:
        for id in marker_list:
                #if marker_located[id] == 0:
                if marker_client.is_marker_available(id) and id in victim_markers:
                    marker_client.send_update('marker', marker_id=int(id), detected=True)
                    drone.send_rc_control(0, 0, 0, 0)
                    print(f"Measuring Marker {id}'s position...")
                    status = f"Measuring Marker {id}'s position..."
                    locate_marker(drone,id)
        rotation += 60
        desired_heading = course + rotation
        desired_heading %= 360  # Keep orientation within 0 to 360 degrees
        if desired_heading > 180:
            desired_heading -= 360  # Convert to -180 to 180 range
        try_count = 0
        while abs(desired_heading - heading) > 20 and not abs(desired_heading + 360 - heading) < 20 and not abs(desired_heading - heading - 360) < 20:
            print(f"Desired heading: {desired_heading}")
            print(f"Actual heading {heading}")
            drone.rotate_clockwise(60)
            time.sleep(1.5)
            marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
            try_count += 1
            if try_count > 5:
                marker_client.send_update('waypoint', marker_id=waypoint_id, detected=False)
                drone.land()
                break
        time.sleep(1)
        

    if abs(heading-course) <= 180:
        drone.rotate_counter_clockwise(heading-course)
    elif heading-course < -180:
        drone.rotate_counter_clockwise(360 + heading-course)
    else:
        drone.rotate_counter_clockwise(heading-course - 360)
    marker_client.send_update('waypoint', marker_id=waypoint_id, detected=True)
'''


def locate_marker(drone, id):
    while True:
        global marker_client, ang, heading, dis, status, marker_located

        marker_client.send_update('marker', marker_id=int(id), detected=True)

        dis[id] = 0
        time.sleep(1)
        ang[id] = 0  # Reset angle for marker #id
        time.sleep(0.5)
        cycle = 0
        status = f"Aligning with marker {id}"
        while abs(ang[id]) > 1:  # Aligning with marker #id
            marker_client.send_update('marker', marker_id=int(id), detected=True)
            if cycle < 2:
                drone.rotate_clockwise(int(ang[id]*1.2))
            else:
                drone.rotate_clockwise(int(ang[id]*1.8))
            cycle += 1
            ang[id] = 0  # Reset angle for each cycle
            time.sleep(1)
            if cycle == 7:
                break
        dis[id] = 0
        print(f"Calculating Marker {id}'s position...")
        status = f"Calculating Marker {id}'s position..."
        marker_client.send_update('marker', marker_id=int(id), detected=True)
        time.sleep(3)
        
        if dis[id] == 0:
            print(f"Marker {id} was lost")
            status = f"Marker {id} was lost"

            marker_located[id] = 0
            marker_client.send_update('marker', marker_id=int(id), detected=False)
            if id in marker_list:
                marker_list.pop(marker_list.index(id))
                break
        else:
            # First check for danger markers and calculate offset BEFORE approaching
            frame = drone.get_frame()
            gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            marker_corners, marker_IDs, _ = cv2.aruco.detectMarkers(
                gray_frame, marker_dict, parameters=param_markers
            )
            
            if marker_IDs is not None and len(marker_IDs) > 0:
                rVec, tVec, _ = aruco.estimatePoseSingleMarkers(
                    marker_corners, MARKER_SIZE, cam_mat, dist_coef
                )
                
                # Just calculate offset here, don't apply it yet

            
            # Now move forward to be close to the marker
            distance_forward = dis[id]
            if distance_forward > 270:
                drone.move_forward(int(distance_forward - 250))
                continue
            else:
                app_pos = uwb_reading(drone)
                drone.move_forward(int(distance_forward))
                time.sleep(5)
                final_pos = uwb_reading(drone)
                if app_pos != [0,0] and final_pos != [0,0]:
                    if abs(distance_forward - np.sqrt((app_pos[0] - final_pos[0])**2 + (app_pos[1] - final_pos[1])**2)) > 100:
                        drone.move_forward(int(distance_forward))

            
            time.sleep(6)
            # marker_client.send_update('waypoint', marker_id=waypoint_id, detected=False)

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
                landing_success = perform_precision_landing(drone, id)
                
                if landing_success:
                    drone.streamoff()
                    drone.reboot()
                    marker_client.send_update('marker', marker_id=int(id), landed=True)
                    status = f"Successfully landed on marker {id}"
                else:
                    status = f"Failed precision landing on marker {id}"
                    # Fall back to regular landing if precision landing fails
                    drone.land()
                    drone.streamoff()
                    drone.reboot()
                    marker_client.send_update('marker', marker_id=int(id), landed=True)
            
            break

def ascend(drone,altitude):
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
            drone.send_command_with_return("downvision 1")
            time.sleep(5)
            switch = False
        
        ret = True
        frame1 = frame_reader.frame
        frame = cv2.cvtColor(frame1, cv2.COLOR_BGR2RGB)

        height = drone.get_height()
        battery = drone.get_battery()
        heading = drone.get_yaw() - start_heading
        if not ret:
            break
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        marker_corners, marker_IDs, reject = cv2.aruco.detectMarkers(
            gray_frame, marker_dict, parameters=param_markers
        )
        
        cv2.putText(frame,f"Height: {height} cm | Heading: {heading} degrees | Course: {course} degrees | Battery: {battery} %",(10,20),cv2.FONT_HERSHEY_PLAIN,0.7,(0,255,255),2)
        cv2.putText(frame,f"Position: X: {round(pos[0], 2)} Y: {round(pos[1], 2)} | Status: {status}",(10,40),cv2.FONT_HERSHEY_PLAIN,0.7,(0,255,255),2)
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
                
                # Highlight danger markers in red
                if ids[0] in danger_markers:
                    marker_color = (0, 0, 255)  # Red for danger markers
                    visible_dangers.append(ids[0])
                else:
                    marker_color = (0, 255, 255)  # Yellow for regular markers
                    visible_markers.append(ids[0])
                
                cv2.polylines(
                    frame, [corners.astype(np.int32)], True, marker_color, 4, cv2.LINE_AA
                )
                corners = corners.reshape(4, 2)
                corners = corners.astype(int)
                top_right = corners[0].ravel()
                top_left = corners[1].ravel()
                bottom_right = corners[2].ravel()
                bottom_left = corners[3].ravel()

                # Calculating the distance
                distance = np.sqrt(
                    tVec[i][0][2] ** 2 + tVec[i][0][0] ** 2 + tVec[i][0][1] ** 2
                )

                scaling_factor = 1
                actual_distance = np.sqrt((scaling_factor*distance)**2 - height**2)

                id = ids[0]
                if ids[0]>= 20:
                    #print(f"id: {ids[0]} is out of list range")
                    continue
                #if dis[ids[0]] == 0:
                    #dis[ids[0]] = actual_distance
                dis[ids[0]] = actual_distance
                angle = 10/(30+(actual_distance-150)/5)*(round(tVec[i][0][0],1))

                ang[ids[0]] = angle
                # Draw the pose of the marker
                point = cv2.drawFrameAxes(frame, cam_mat, dist_coef, rVec[i], tVec[i], 4, 4)
                
                # Add marker type label (valid or danger)
                marker_type = "DANGER" if ids[0] in danger_markers else "Target"
                cv2.putText(
                    frame,
                    f"id: {ids[0]} ({marker_type}) Dist: {round(actual_distance, 2)} Ang: {round(angle,2)}",
                    top_right,
                    cv2.FONT_HERSHEY_PLAIN,
                    1.3,
                    marker_color,
                    2,
                    cv2.LINE_AA,
                )

                cv2.putText(
                    frame,
                    f"x:{round(tVec[i][0][0],1)} y: {round(tVec[i][0][1],1)} ",
                    bottom_right,
                    cv2.FONT_HERSHEY_PLAIN,
                    1.0,
                    marker_color,
                    2,
                    cv2.LINE_AA,
                )
            
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
                
                # print(ids, "  ", corners)


        #cv2.imshow("frame", frame)
    
        # Check if streaming readiness hasn't been signaled yet
        if not stream_ready.is_set():
            # Signal that video streaming is ready
            stream_ready.set()
            print("Event Signal Set: Stream is live.")

        if cv2.waitKey(1) & 0xFF == ord('z'):
            break
        
        drone.set_frame(frame1)

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