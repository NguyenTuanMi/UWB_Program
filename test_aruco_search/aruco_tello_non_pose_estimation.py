import cv2 
import numpy as np

cap = cv2.VideoCapture(0)
marker_color = (0, 255, 255)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, params)

aruco_side_length = 0.05 #in meters
camera_matrix = np.array([ #this is just an example matrix
    [919.654, 0, 479.5], 
    [0, 919.654, 269.5], 
    [0, 0, 1]
    ])

dist_coeffs = np.array([0, 0, 0, 0, 0]) #assuming no lens distortion

object_points = np.array([
    [-aruco_side_length/2,  aruco_side_length/2, 0],
    [ aruco_side_length/2,  aruco_side_length/2, 0],
    [ aruco_side_length/2, -aruco_side_length/2, 0],
    [-aruco_side_length/2, -aruco_side_length/2, 0]], dtype=np.float32)
while True:
    ret, img = cap.read()
    if not ret:
        break
    
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    #img = cv2.flip(img, 1)

    corners, ids, rejected = detector.detectMarkers(gray)
    #cv2.polylines(
    #                img, [corners.astype(np.int32)], True, marker_color, 4, cv2.LINE_AA
    #           )
    if ids is not None:
        for i in range(0, len(ids)): 
            retVal, rvecs, tvecs = cv2.solvePnP(object_points, corners[i], camera_matrix, dist_coeffs)
            cv2.drawFrameAxes(img, cameraMatrix=camera_matrix, distCoeffs=dist_coeffs, rvec=rvecs, tvec=tvecs, length=0.1)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    cv2.imshow("Aruco Detection", img)

cap.release()
cv2.destroyAllWindows()