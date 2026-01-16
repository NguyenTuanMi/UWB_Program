"""
Connect to drone for this simple example!

Alternatively, you may try example_nodrone.py without connecting to a drone.

IMPT: Run this after running swarmserverclient.py, then observe the GUI.
"""

from swarmserverclient import MarkerClient
from djitellopy import Tello
import time

tello = Tello()

tello.connect()
start_batt = tello.get_battery()
status_str = f"Waiting for takeoff. Start Batt %: {start_batt}"

marker_client = MarkerClient(drone_id=1)
marker_client.client_takeoff_simul([99], status_message=status_str)     # [99] ensures that takeoff is only triggered by the user. Otherwise, put the list of drone_ids you want to takeoff with.
tello.takeoff()

marker_client.send_update('marker', 1, detected=True)
time.sleep(2)
marker_client.send_update('marker', 1, landed=True)
time.sleep(2)

tello.end()
marker_client.send_update('status', status_message='Landed')