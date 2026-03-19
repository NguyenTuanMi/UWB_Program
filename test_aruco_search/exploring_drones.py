from shared_utils.customtello import CustomTello
from pathlib import Path
import time, sys
import socket
from UWB_Manipulation.UWB_Reader import get_target_position # own custom library
from shared_utils.shared_utils import *
import cv2 
import socket
import threading
import json

RC_SPEED_SCALE = 0.4
PERIOD_S = 0.1
UWB_OFFSET: tuple[float] = (0, 0) 
POSITION = [0,0]
DOWN_CENTERING_THRESHOLD = 20
CONSECUTIVE_THRESHOLD = 3

workspace_root = Path(__file__).resolve().parent.parent
sys.path.append(str(workspace_root))

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

def checkMarkerCenter(frame_center, marker_center):
    frame_center_x = frame_center[0]
    frame_center_y = frame_center[1]

    marker_center_x = marker_center[0]
    marker_center_y = marker_center[1]
    
    dx = frame_center_x - marker_center_x
    dy = frame_center_y - marker_center_y
    return abs(dx) <= DOWN_CENTERING_THRESHOLD and abs(dy) <= DOWN_CENTERING_THRESHOLD

def getCenter(frame, corner):
    if frame is None or frame.size == 0:
        return None, None, frame
    pic = np.array(frame, copy=True)
    corner = np.array(corner, copy=True)
    
    frame_center_x = pic.shape[1]//2
    frame_center_y = pic.shape[0]//2

    corner.reshape((4,2))
    (top_left, top_right, bottom_right, bottom_left) = corner[0]
    marker_center_x = int((top_left[0]+bottom_right[0])//2)
    marker_center_y = int((top_right[1]+bottom_left[1])//2)
    
    return (frame_center_x, frame_center_y), (marker_center_x, marker_center_y)

def calculate_rc_values(marker_center, frame_center):
    """Calculate RC control values based on marker position"""
    dx = marker_center[0] - frame_center[0]
    dy = marker_center[1] - frame_center[1]
    
    # Scale the RC values based on pixel distance
    lr = int(dx * RC_SPEED_SCALE)
    fb = int(dy * RC_SPEED_SCALE)

    lr = int(np.clip(lr, -30, 30))
    fb = int(np.clip(fb, -30, 30))
    
    return lr, fb

def movementThread(drone, frame_read):
    DELAY = 3
    drone.takeoff()
    has_been_detected = False
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    broadcast_address = "255.255.255.255"
    port = 6000
    # Allow socket reuse
    while True: 
        #retry_count = 0
        img = frame_read.frame
        '''
        while img is None and retry_count <= 3:
            img = frame_read.frame
            if img is None or img.size == 0:
                time.sleep(0.1)
                retry_count += 1
        '''
        if has_been_detected:
            break
        if img is None:
            continue
        img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE)
        frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        #cv2.imshow("Frame", frame) 
        corners, ids, rejected = detector.detectMarkers(frame)
        consecutive_frame = 0  
        if ids is not None:
            print("Detected IDs:", ids.flatten())
            drone.send_rc_control(0,0,0,0)
            time.sleep(DELAY)
            
            for i in range(0, len(ids)):
                frame_center, marker_center = getCenter(frame, corners[i])
                print(f"Frame Center {frame_center}")
                print(f"Marker Center {marker_center}")
                if checkMarkerCenter(frame_center, marker_center):
                    drone.send_rc_control(0,0,0,0)
                    time.sleep(3)
                    print("It has arrived")
                    #message = 'hello'
                    pose = get_target_position(11)
                    print("Sending", pose)
                    pose = json.dumps(pose).encode()
                    retries = 0
                    while retries <= 10:
                        sock.sendto(pose, (broadcast_address, port)) 
                        retries += 1
                    has_been_detected = True
                    drone.land() 
                    break
                    #consecutive_frame += 1
                else:
                    #consecutive_frame = 0
                    left_right, forward_back= calculate_rc_values(marker_center, frame_center)
                    drone.send_rc_control(left_right, -forward_back, 0, 0)
                    time.sleep(0.4)
                print(f"Number of Consecutive frame: {consecutive_frame}")
            
        else: 
            drone.send_rc_control(0, 10, 0, 0)
        
        print(f"Is frame None {frame is None}") 
    
def main():
    params = load_params()

    tello = CustomTello(network_config=params.NETWORK_CONFIG)
    TAKEOFF_DELAY = 3
    DELAY = 3

    tello.connect()
    tello.streamon()
    tello.send_command_with_return("downvision 1")
    time.sleep(TAKEOFF_DELAY)
    time.sleep(DELAY)
    frame_read = tello.get_frame_read()
    movementHandler = threading.Thread(target=movementThread, args=(tello,frame_read,), daemon=True)
    movementHandler.start()

    cv2.namedWindow("Tello Camera", cv2.WINDOW_NORMAL)
    
    while True: 
        frame = frame_read.frame
        if frame is None:
            continue
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        corners, ids, rejected = detector.detectMarkers(frame)
        if ids is not None: 
            for i in range(0, len(ids)):
                frame_center, marker_center = getCenter(frame, corners[i])

                # ================== NEWLY ADDED ==================
                corners[i].reshape((4,2))
                (top_left, top_right, bottom_right, bottom_left) = corners[i][0]
                top_right = (int(top_right[0]), int(top_right[1]))
                bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
                bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
                top_left = (int(top_left[0]), int(top_left[1]))
    
                cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
                cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
                cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
                cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)
                # =================================================

                cv2.circle(frame, marker_center, 2, (255,0,0), -1)
                cv2.circle(frame, frame_center, 2, (0,0,255), -1)
        cv2.imshow("Tello Camera", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            tello.land()
            break
    cv2.destroyAllWindows()
    tello.streamoff()
    tello.end()
    movementHandler.join(timeout=1)

main()   



            
            


    
    



