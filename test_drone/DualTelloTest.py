from djitellopy import Tello
import cv2
import time
import threading
from .CustomTello import *
from .const import *

tello7 = CustomTello(TELLO_7_NETWORKCONFIG)
tello8 = CustomTello(TELLO_8_NETWORKCONFIG)



def flight_plan(tello, stop_flag):
    """Run flight commands while the main thread handles video display."""
    try:
        tello.takeoff()
        tello.move_forward(100)
        tello.rotate_clockwise(180)
        tello.move_back(100)  # 'back' to return
        tello.land()
    except Exception as e:
        print("Flight error:", e)
        try:
            tello.land()
        except:
            pass
    finally:
        # signal main loop to stop video when done
        stop_flag["stop"] = True

def main():
    tello = Tello()
    stop_flag = {"stop": False}

    try:
        tello.connect()
        print(f"Battery: {tello.get_battery()}%")

        # Start video stream
        tello.streamon()
        time.sleep(1)  # give stream a moment to initialize
        frame_read = tello.get_frame_read()  # starts its own reader thread

        # Launch flight in background
        t = threading.Thread(target=flight_plan, args=(tello, stop_flag), daemon=True)
        t.start()

        # === MAIN THREAD: show live video ===
        cv2.namedWindow("Tello Live", cv2.WINDOW_NORMAL)
        while not stop_flag["stop"]:
            frame = frame_read.frame
            frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            if frame is not None:
                cv2.imshow("Tello Live", frame)
            # Press 'q' to quit early
            if cv2.waitKey(1) & 0xFF == ord('q'):
                stop_flag["stop"] = True
                break
            time.sleep(0.005)
        # ====================================

    except Exception as e:
        print("Error:", e)
        try:
            tello.land()
        except:
            pass
    finally:
        try:
            tello.streamoff()
        except:
            pass
        tello.end()
        cv2.destroyAllWindows()
        
main()