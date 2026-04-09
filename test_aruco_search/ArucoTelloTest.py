from djitellopy import Tello
import cv2
from cv2 import aruco
import threading 
import time
import numpy as np

djiTello = Tello()

djiTello.connect()
time.sleep(3)
print("battery", djiTello.get_battery())

djiTello.streamon()
frame_read = djiTello.get_frame_read()

def get_calibration_parameters(controller):
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

def draw_pose_axes(frame, corners, ids, rvecs, tvecs):
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
        
        cv2.putText(frame, f"X:{x_cm:+.1f} Y:{y_cm:+.1f} Z(fwd):{z_cm:.1f} H:{horiz_cm:.1f}",
                    (10, 30 + 22*i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,0), 2)
    return frame

def detect_marker_pose(gray_frame,
                       aruco_dict_type=cv2.aruco.DICT_5X5_250,
                       marker_size_m=0.19):
    K = np.array([
            [232.08608036, 0.000000, 152.2358733], 
            [0.000000, 232.64995134, 124.98737218], 
            [0.000000, 0.000000, 1.000000]
        ])
    
    D = np.array([5.26408126e-01, -1.52051597e+00, 1.20937906e-02, -1.34771857e-03, 1.29077550e+00])

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
        print("Some errors encountered")
    return corners. rvecs, tvecs

def flight(tello):
    while True:
        frame = frame_read.frame
        if frame is None or not hasattr(frame, "shape") or frame.size == 0:
            time.sleep(0.02)
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rvecs, tvecs = detect_marker_pose(gray)

        cv2.drawFrameAxes()
        mc = np.mean(corners[i][0], axis=0).astype(int)
        cv2.circle(frame, (mc[0], mc[1]), 5, (0, 0, 255), -1)

        
        horiz_cm = (x_cm**2 + z_cm**2) ** 0.5
        
        cv2.putText(frame, f"X:{x_cm:+.1f} Y:{y_cm:+.1f} Z(fwd):{z_cm:.1f} H:{horiz_cm:.1f}",
                    (10, 30 + 22*i), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0,255,0), 2)
    tello.streamoff()
thread = threading.Thread(target=flight, args=(djiTello,), daemon=True)
thread.start()

keep_recording = True
cv2.namedWindow("Winname")
while keep_recording:
    pic = frame_read.frame
    pic = cv2.cvtColor(pic, cv2.COLOR_BGR2RGB)
    cv2.imshow("Tello stream", pic)

    if cv2.waitKey(1) & 0xFF == ord("a"):
        keep_recording=False
        break

cv2.destroyAllWindows()    
djiTello.end()

