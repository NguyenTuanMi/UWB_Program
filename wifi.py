from djitellopy import Tello
import time

# Connect to Tello
tello = Tello()
tello.connect()

tello.send_control_command("wifisetchannel 6") 