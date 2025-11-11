import json, math, time
from .constants import *

def validate_waypoints(json_filename):
    with open(json_filename, 'r') as f:
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

def save_pos(waypoints_list, orientations_list, abs_pos, orientation, distance=0):
    """Update the drone's abs_pos (in cm) and orientation, and store it in the respective waypoints/orientations lists. Can be used for dead reckoning only (not UWB - TBC)"""
    rad = math.radians(orientation)
    abs_pos["x_cm"] += int(distance * math.sin(rad))
    abs_pos["y_cm"] += int(distance * math.cos(rad))
    # waypoints_list.append({"x": abs_pos["x_cm"], "y": abs_pos["y_cm"], "orientation": orientation, "distance": distance})
    # print("[INFO] Current abs_pos:", {"x": abs_pos["x"], "y": abs_pos["y"], "orientation": orientation, "distance": distance})
    waypoints_list.append([abs_pos["x_cm"], abs_pos["y_cm"]])
    orientations_list.append(orientation)

def obtain_orientation(waypoints_list): # mostly for UWB

    if len(waypoints_list) < 2:
        print("Not enough positions in positions_list to calculate orientation.")   # should be only for START_HEADING
        orientation = START_HEADING
    else:
        pos1 = waypoints_list[-2]
        pos2 = waypoints_list[-1]

        # Calculate the change in position
        delta_x = pos2[0] - pos1[0]
        delta_y = pos2[1] - pos1[1]
        
        # Calculate the angle in degrees
        orientation = math.degrees(math.atan2(delta_y, delta_x))
        
        # Normalize orientation to 0 (forward) and negative for clockwise
        orientation = (orientation + 360) % 360  # Normalize to [0, 360)
        if orientation > 180:
            orientation -= 360  # Convert to [-180, 180)

    return orientation

def save_pos_UWB(waypoints_list_UWB, orientations_list_UWB, UWB_pos):
    """Update the drone's UWB_pos (in cm) and orientation, and store it in the respective waypoints/orientations lists."""
    waypoints_list_UWB.append(UWB_pos)

    orientation = obtain_orientation(waypoints_list_UWB)
    orientations_list_UWB.append(orientation)

def save_errors(pos_error_list, orientation_error_list, abs_pos, UWB_pos, orientation, UWB_orientation):
    """
    Calculate the positional error between two points (absolute position and UWB position).
    Supports both 2D and 3D coordinates.

    :param abs_pos: Tuple or list of absolute position (x, y) or (x, y, z).
    :param UWB_pos: Tuple or list of UWB position (a, b) or (a, b, c).
    :return: Tuple containing the positional error (x-a, y-b) or (x-a, y-b, z-c).
    """
    if len(abs_pos) != len(UWB_pos):
        raise ValueError("Both coordinates must have the same dimensions (2D or 3D).")
    
    pos_error = tuple(abs_pos[i] - UWB_pos[i] for i in range(len(abs_pos)))
    pos_error_list.append(pos_error)

    orientation_error = orientation - UWB_orientation
    orientation_error_list.append(orientation_error)


def check_error_large(error, threshold=50):
    """
    Check if the magnitude of the last error in errors_list is more than a threshold (default: 50 cm).
    
    :param errors_list: List of error tuples.
    :param threshold: The magnitude threshold in cm.
    :return: Boolean indicating whether the magnitude exceeds the threshold.
    """
    magnitude = math.sqrt(sum(e**2 for e in error))  # Calculate the magnitude of the vector
    return magnitude > threshold

def correct_pos_error(errors_list):
    """
    Calculate the positional error between two points (absolute position and UWB position).
    Supports both 2D and 3D coordinates.

    :param abs_pos: Tuple or list of absolute position (x, y) or (x, y, z).
    :param UWB_pos: Tuple or list of UWB position (a, b) or (a, b, c).
    :return: Tuple containing the positional error (x-a, y-b) or (x-a, y-b, z-c).
    """
    if check_error_large(errors_list[-1]):
        pass # TBD 8 Jan


def check_mission_pad_id(tello, land_id = 0):
    """Check for mission pad ID and print it if detected."""
    pad_id = tello.get_mission_pad_id()
    if pad_id != -1:
        print(f"Detected Mission Pad ID: {pad_id}")
        if pad_id == LAND_ID:
            tello.go_xyz_speed_mid(0, 0, 50, 50, LAND_ID)
            time.sleep(2)
            FLYING_STATE = False
            tello.end()

def calculate_vector(pos1, pos2, dims=2):
    """
    Calculate the 2D/3D vector from pos1 to pos2.  Default: 2D
    :param pos1: List [x1, y1, z1] representing the first position.
    :param pos2: List [x2, y2, z2] representing the second position.
    :return: List representing the vector [dx, dy, dz].
    """
    if len(pos1) != dims or len(pos2) != dims:
        raise ValueError("Positions must be 3D coordinates [x, y, z].")
    
    vector = [round((pos2[i] - pos1[i]),2) for i in range(dims)]
    print(f"{dims}D Vector from {pos1} to {pos2}: {vector}")
    return vector

def calculate_distance(pos1, pos2, dims=2):
    """
    Calculate the distance between pos1 and pos2. Default: 2D
    Units: cm (both input and output)
    :param pos1: List [x1, y1, z1] representing the first position.
    :param pos2: List [x2, y2, z2] representing the second position.
    :return: Float representing the Euclidean distance.
    """
    if len(pos1) != dims or len(pos2) != dims:
        raise ValueError("Positions must be 3D coordinates [x, y, z].")
    
    distance = math.sqrt(sum((pos2[i] - pos1[i])**2 for i in range(dims)))
    rounded_distance = round(distance,2)
    print(f"{dims}D Distance from {pos1} to {pos2}: {rounded_distance}cm")
    return rounded_distance

def printdistance(prevpos, currentpos, dims=2):
    try:
        calculate_vector(prevpos,currentpos,dims)
        calculate_distance(prevpos,currentpos,dims)
    except IndexError:
        print("Not enough points.")
        return 0
    except Exception as e:
        print(e)
    return