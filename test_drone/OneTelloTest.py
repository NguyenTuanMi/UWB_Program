from djitellopy import Tello
import cv2
import threading 
import time

djiTello = Tello()

djiTello.connect()
time.sleep(3)
print("battery", djiTello.get_battery())

djiTello.streamon()
frame_read = djiTello.get_frame_read()

def flight(tello):
    tello.takeoff()
    tello.move_up(50)
    tello.rotate_clockwise(270)
    tello.land()
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

