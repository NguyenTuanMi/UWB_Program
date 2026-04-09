#Program for path finding and alignment. 
#Written by Vincent Gaudeo Lie, modified by Nguyen Tuan Minh
from .constants import *
from matplotlib.path import Path
from pathlib import Path

import math
from time import time
from shared_utils.customtello import CustomTello
from shared_utils.shared_utils import *
import socket
from UWB_Manipulation.UWB_Reader import get_target_position
from pathfinding import enlarge, draw_path, find_path
import json
import threading

workspace_root = Path(__file__).resolve().parent.parent
sys.path.append(str(workspace_root))
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
        orientation = drone.get_yaw()
        abs_position = {"x_cm": lastpos_cm[0], "y_cm": lastpos_cm[1]}  # in cm; FOR DEAD RECKONING, updated using save_pos()
        #save_pos_UWB(waypoints_UWB, orientations_UWB, lastpos_cm[0:2])

        target_x = target_position[0] * 100  # Convert to cm
        target_y = target_position[1] * 100  # Convert to cm
        
        delta_x = target_x - abs_position["x_cm"]
        delta_y = target_y - abs_position["y_cm"]
        orientation = math.degrees(math.atan2(delta_y, delta_x)-orientation)

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

safe_area = [(58.59, 155.69), (110.11, 156.19), (110.11, 146.94), (58.39, 146.53)] # Starting area on the playing field
safe_area = enlarge(safe_area, -0.7)
    
def get_marker_position(port, max_retries=10, timeout=0.2):
    GOT_POS = False
    retry_count = 0

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow socket reuse

    server_address = ('0.0.0.0', port)  # Bind to all available interfaces
    sock.bind(server_address)  # Bind the socket to the address and port
    sock.settimeout(None)  # Set the timeout
    while not GOT_POS:
        # Attempt to receive data
        data, address = sock.recvfrom(4096)
        if data is not None: 
            pos = json.loads(data.decode())
            print(f"Receiverd List of Positions: {pos}")
            # Process the received data
            GOT_POS = True
            sock.close()
            pos = pos[0]
            return pos


def hovering(drone):
    global hovering_status
    while True: 
        drone.send_rc_control(0,0,0,0)
        if not hovering_status:
            break

def wait_for_marker():
    port = 6000
    global position,  hovering_status
    end_pos = get_marker_position(port)
    time.sleep(2)
    init_pos = get_target_position(0)
    time.sleep(2)

    path = find_path(init_pos, end_pos)
    position = path
    hovering_status = False

def main():
    TAKEOFF_DELAY = 5
    port = 6000
    
    params = load_params()
    global position, hovering_status
    hovering_status = True
    tello = CustomTello(network_config=params.NETWORK_CONFIG)
    tello.connect()
    
    print(f"Battery: {tello.get_battery()}")
    time.sleep(0.5)

    tello.takeoff()
    time.sleep(TAKEOFF_DELAY)
    t1 = threading.Thread(target=wait_for_marker, daemon=True)
    t2 = threading.Thread(target=hovering, args=(tello,), daemon=True)
    t1.start()
    t2.start()
    
    t1.join()
    t2.join()

    time.sleep(0.5)

    P = draw_path(position, get_intermitten = True, draw_intermitten = True, WANT_TO_MOVE = True, drone=tello)
    time.sleep(2
               )
    print(f"Landing Now. End Battery: {tello.get_battery()}%")
    tello.land()
    tello.end()

if __name__ == "__main__":
    main()
#path = find_path(START_POINT, (70, 150))


