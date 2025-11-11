from .utils import *
from .constants import *

import json, math, time, sys, argparse
from pathlib import Path

#from shared_utils.customtello import MockTello
from shared_utils.shared_utils import *
from shared_utils.customtello import CustomTello

import socket

workspace_root = Path(__file__).resolve().parent.parent
sys.path.append(str(workspace_root))

from UWB_Manipulation.UWB_Reader import get_target_position # own custom library

PERIOD_S = 0.1
UWB_OFFSET: tuple[float] = (0, 0) 

def execute_waypoints(drone, target_position, simulate = False):
    """
    Connect, takeoff and landing should be done outside this function.
    TODO 18 Feb: Move to shared_utils
    """
    global waypoints
    global start_batt, end_batt
    tello = drone
    DELAY = 2
    
    try:      
        lastpos_cm = [int(round(coord*100,0)) for coord in get_target_position(params.UWBTAG_ID)] #*100 to convert to cm
        orientation = START_HEADING
        abs_position = {"x_cm": lastpos_cm[0], "y_cm": lastpos_cm[1]}  # in cm; FOR DEAD RECKONING, updated using save_pos()
        #save_pos_UWB(waypoints_UWB, orientations_UWB, lastpos_cm[0:2])

        target_x = target_position[0] * 100  # Convert to cm
        target_y = target_position[1] * 100  # Convert to cm
        
        delta_x = target_x - abs_position["x_cm"]
        delta_y = target_y - abs_position["y_cm"]
        orientation = math.degrees(math.atan2(delta_y, delta_x))

        orientation = (orientation + 360) % 360  # Normalize to [0, 360)
        if orientation > 180:
            orientation -= 360  # Convert to [-180, 180)
        orientation = int(round(orientation))
        if abs(orientation) > 10:
            if orientation > 0:
                tello.rotate_clockwise(abs(orientation))
            else: 
                tello.rotate_counter_clockwise(orientation)
        time.sleep(DELAY)

        distance = int(round(math.sqrt(delta_x**2 + delta_y**2)))
        # Execute each waypoint
        print("Calculated distance", distance)
        while distance > INCREMENT_CM:
            if distance - INCREMENT_CM < 20:
                print("[INFO] Distance fine split. Remaining:", distance)
                tello.move_forward(50)
                distance -= 50
                time.sleep(DELAY)

            else:
                print("[INFO] Distance split. Remaining:", distance)                    
                tello.move_forward(INCREMENT_CM)
                distance -= INCREMENT_CM
                time.sleep(DELAY)
                
            # Move remaining distance (if between 50 and 100 cm)
        if distance != 0:
            print("[INFO] No split required. Remaining:", distance)
            tello.move_forward(distance)
            time.sleep(DELAY)
        else:   # for distance = 0
            pass
            print("[INFO] Distance remaining = 0. Path completed.")
            # Handle forward movement in increments
            # NEW 7 JAN - READS UWB DISTANCE AFTER EVERY MAJOR WAYPOINT          
    
    except Exception as e:
        print(f"Error occurred: {e}")
    
    finally:
        print("Mission completed! Not Landing.")

def get_marker_position(port, max_retries=3, timeout=0.2):
    GOT_POS = False
    retry_count = 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow socket reuse

    server_address = ('0.0.0.0', port)  # Bind to all available interfaces
    sock.bind(server_address)  # Bind the socket to the address and port
    sock.settimeout(timeout)  # Set the timeout

    while not GOT_POS and retry_count < max_retries:
        try:
            # Attempt to receive data
            data, address = sock.recvfrom(4096)
            lines = data.decode().splitlines()

            line = lines[0]
            # Process the received data
            GOT_POS = True
            parts = line.split(',')
            pos = (float(parts[2])+UWB_OFFSET[0] , float(parts[3])+UWB_OFFSET[1], float(parts[4]))
            print(f"Target {port}: {pos}")
            sock.close()
            return pos

            # Increment retry count if no matching target_id found
        except socket.timeout:
            # Increment retry count on timeout
            print(f"[WARNING] Timeout occurred. Retry {retry_count + 1} of {max_retries}.")
            retry_count += 1

        except Exception as e:
            # Handle other exceptions
            print(f"Error parsing data: {e}")
            retry_count += 1
def main():
    params = load_params()

    tello = CustomTello(network_config=params.NETWORK_CONFIG)
    TAKEOFF_DELAY = 3
    DELAY = 2

    tello.connect()
    start_batt = tello.get_battery()
    print(f"Battery: {tello.get_battery()}")
    time.sleep(0.5)
    
    # Take off
    print("Taking off...")
    tello.takeoff()
    time.sleep(TAKEOFF_DELAY)  # Give more time for takeoff to stabilize

    detected_marker = False
    while True:
        #detected_marker = detect_marker(tello, params.MARKER_ID)
        try:
            port = 600
            pos = get_marker_position(port)
            pos = [1.88, 4.34]
            time.sleep(2)

            execute_waypoints(tello, pos, params.NO_FLY)
            print(f"Landing Now. End Battery: {tello.get_battery()}%")
            tello.land()
            tello.end()
            break   
        
        except Exception as e:
            # Handle other exceptions
            print(f"Error parsing data: {e}")
        except socket.timeout:
            print("Position marker not detected yet. Retrying in 2 seconds...")
main()