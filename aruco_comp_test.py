import cv2 
import numpy as np

cap = cv2.VideoCapture(0)
marker_color = (0, 255, 255)
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
params = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, params)
DOWN_CENTERING_THRESHOLD = 10

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
    
    top_right = (int(top_right[0]), int(top_right[1]))
    bottom_right = (int(bottom_right[0]), int(bottom_right[1]))
    bottom_left = (int(bottom_left[0]), int(bottom_left[1]))
    top_left = (int(top_left[0]), int(top_left[1]))
    
    cv2.line(frame, top_left, top_right, (0, 255, 0), 2)
    cv2.line(frame, top_right, bottom_right, (0, 255, 0), 2)
    cv2.line(frame, bottom_right, bottom_left, (0, 255, 0), 2)
    cv2.line(frame, bottom_left, top_left, (0, 255, 0), 2)

    marker_center_x = int((top_left[0]+bottom_right[0])//2)
    marker_center_y = int((top_right[1]+bottom_left[1])//2)
    
    return (frame_center_x, frame_center_y), (marker_center_x, marker_center_y)

while True:
    ret, img = cap.read()
    if not ret:
        break
    
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)

    corners, ids, rejected = detector.detectMarkers(gray)
    if ids is not None:
        for i in range(0, len(ids)): 
            marker_center, frame_center = getCenter(img, corners[i])
            cv2.circle(img, marker_center, 2, (255,0,0), -1)
            cv2.circle(img, frame_center, 2, (0,0,255), -1)
            val = checkMarkerCenter(frame_center, marker_center)
            print("Is center?", val)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    cv2.imshow("Aruco Detection", img)

cap.release()
cv2.destroyAllWindows()