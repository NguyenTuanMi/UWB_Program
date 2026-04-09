from djitellopy import Tello
from shared_utils.customtello import CustomTello
import time
import cv2

pi_id = 5
network_config = {
    'host': f'192.168.0.{100+pi_id}',
    'control_port': 9000 + pi_id,
    #'control_port': 9020 + pi_id,
    'state_port': 8000 + pi_id,
    'video_port': 11100 + pi_id
}
tello = CustomTello(network_config=network_config)

tello.connect()
start_batt = tello.get_battery()
status_str = f"Waiting for takeoff. Start Batt %: {start_batt}"
tello.turn_motor_on()
time.sleep(3)
#tello.land()
tello.end()
