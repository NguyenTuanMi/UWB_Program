import cv2
import cv2.aruco as aruco
import numpy as np
import time
from shared_utils.customtello import CustomTello
#from UWB_Wrapper.UWB_ReadUDP import get_target_position

# ================== CONFIG ==================
NETWORK_CONFIG = {
    "host": "192.168.0.105",  # Raspberry Pi's IP address
    "control_port": 9005,     # Default Tello control port
    "state_port": 8005,       # Default Tello state port
    "video_port": 11105       # Default Tello video port
}
MARKER_ID = 2  # ArUco marker ID to detect
MARKER_SIZE_M = 0.19  # Marker side length in meters
ARUCO_DICT = aruco.DICT_5X5_100  # ArUco dictionary type

# ================== SETUP ARUCO DETECTOR ==================
aruco_dict = aruco.getPredefinedDictionary(ARUCO_DICT)
params = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, params)

# ================== TELLO SETUP ==================
tello = CustomTello(NETWORK_CONFIG)
tello.connect()
print("Battery:", tello.get_battery(), "%")
#tello.streamoff()
time.sleep(0.5)
tello.streamon()
tello.send_command_with_return("downvision 1")
frame_reader = tello.get_frame_read()

# ================== TAKEOFF ==================
print("[INFO] Taking off...")
tello.takeoff()
time.sleep(2)

# ================== MAIN LOOP ==================
print("[INFO] Flying forward and searching for ArUco marker...")
while True:
    frame = frame_reader.frame
    if frame is None:
        continue

    # Rotate bottom camera feed 90° clockwise
    frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    # Convert to grayscale for ArUco detection
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = detector.detectMarkers(gray)

    if ids is not None and MARKER_ID in ids.flatten():
        i = list(ids.flatten()).index(MARKER_ID)
        c = corners[i][0]

        # Marker center
        mx, my = c[:, 0].mean(), c[:, 1].mean()
        # Image center
        cx, cy = frame.shape[1] // 2, frame.shape[0] // 2
        # Errors
        err_x = mx - cx
        err_y = my - cy

        # Draw marker and display errors
        aruco.drawDetectedMarkers(frame, [corners[i]])
        cv2.circle(frame, (int(mx), int(my)), 5, (0, 255, 0), -1)
        cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
        text = f"err_x={err_x:.1f}, err_y={err_y:.1f}"
        cv2.putText(frame, text, (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        print(text)

        # Stop moving and hover
        print("[INFO] Marker detected. Hovering...")
        tello.send_rc_control(0, 0, 0, 0)  # Stop all movement
        break
    else:
        # Move forward continuously
        tello.send_rc_control(0, 20, 0, 0)  # Move forward at speed 20

    # Display the frame
    cv2.imshow("Bottom Camera View", frame)

    # Exit on ESC key
    key = cv2.waitKey(1) & 0xFF
    if key == 27:  # ESC
        break

'''
# ================== PUBLISH POSITION ==================
print("[INFO] Publishing positional data...")
for _ in range(10):  # Publish position 10 times
    # Retrieve UWB position data
    #uwb_position = get_target_position(target_id=0)  # Replace with the correct tag ID
    uwb_position = [0,0,0]
    if uwb_position:
        x, y, z = uwb_position
        print(f"[INFO] UWB Position: x={x}, y={y}, z={z}")
        tello.uwb_publisher.publish_position(x, y, z)  # Publish UWB position over UDP
    else:
        print("[WARNING] Failed to retrieve UWB position.")
    time.sleep(1)
'''

# ================== CLEANUP ==================
cv2.destroyAllWindows()
tello.streamoff()
tello.land()
tello.end()