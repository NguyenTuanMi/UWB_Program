import cv2 
import djitellopy as tello

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

djiTello = tello.Tello()
djiTello.connect()
print("Battery:", djiTello.get_battery())

djiTello.streamon()
frame_read = djiTello.get_frame_read()

cv2.namedWindow("Tello Stream")
while True:
    img = frame_read.frame
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    frame = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
    corners, ids, rejected = detector.detectMarkers(frame)
    if ids is not None:
        cv2.aruco.drawDetectedMarkers(img, corners, ids)
        print("Detected IDs:", ids.flatten())
    cv2.imshow("Tello Stream", img)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        djiTello.streamoff()
        break

cv2.destroyAllWindows()
djiTello.end()
