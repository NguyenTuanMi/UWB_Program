"""
This is just for simulation, and communicates with the server without any actual drone commands - No need to connect to drone for this to work!
print() statements indicate where to place your drone functions. 

You may try example.py with a real drone.

IMPT: Run this AFTER running swarmserverclient.py, then observe the GUI.
"""

from swarmserverclient import MarkerClient
import time, threading



marker_client = MarkerClient(drone_id=1)
marker_client2 = MarkerClient(drone_id=2)

print("Simulating waypoint 1 occupied by another drone.")   
marker_client.send_update('waypoint', 1, detected=True)     # This simulates a waypoint being locked, which will unlock after X seconds (see check_timeout under MarkerServer)
time.sleep(1)

def nav_thread(marker_client: MarkerClient):
    status_str = f"Waiting for takeoff. Start Batt %: XXX"
    marker_client.client_takeoff_simul([99], status_message=status_str) # [99] ensures that takeoff is only triggered by the user. Otherwise, put the list of drone_ids you want to takeoff with.
    print("drone.takeoff() here")
    marker_client.send_update('status', status_message='Taking off!')
    while True:
                # Part 1: Marker Updates
        marker_client.send_update('marker', 1, detected=True)
        time.sleep(2)
        marker_client.send_update('marker', 1, landed=True)
        time.sleep(2)
        marker_client.send_update('marker', 3, detected=True)
        time.sleep(2)
        marker_client.send_update('marker', 4, detected=True)

        # Part 2: Waypoint Updates
        if marker_client.is_waypoint_available(1):
            marker_client.send_update('waypoint', 1, detected=True)
            marker_client.send_update('status', status_message='Heading to waypoint 1.')
            print("Heading to waypoint 1. Insert drone navigation/search code here.")
            time.sleep(5)
            print("Waypoint 1 completed.")
            marker_client.send_update('status', status_message='Waypoint 1 completed.')
            marker_client.send_update('waypoint', 1, detected=False)
        else:
            print("Unable to head to waypoint 1. Insert wait and check again is_waypoint_available.")
            marker_client.send_update('status', status_message="Unable to head to waypoint 1. Holding.")

        if marker_client.is_waypoint_available(2):
            marker_client.send_update('waypoint', 2, detected=True, status_message='Heading to waypoint 2.')
            marker_client.send_update('status', status_message='Heading to waypoint 2.')
            print("Heading to waypoint 2. Insert drone navigation/search code here.")
            time.sleep(2)
            print("Waypoint 2 completed.")
            marker_client.send_update('status', status_message='Waypoint 2 completed.')
            marker_client.send_update('waypoint', 2, detected=False)
        else:
            print("Unable to head to waypoint 2. Insert wait and check again is_waypoint_available.")
            marker_client.send_update('status', status_message="Unable to head to waypoint 2. Holding.")

nav_thread1 = threading.Thread(target=nav_thread, args=(marker_client,))
nav_thread2 = threading.Thread(target=nav_thread, args=(marker_client2,))
nav_thread1.start()
time.sleep(1)
nav_thread2.start()

# try:
#     # This while-loop simulates some updates being sent to the server.
#     while True:

#         # Part 1: Marker Updates
#         marker_client.send_update('marker', 1, detected=True)
#         time.sleep(2)
#         marker_client.send_update('marker', 1, landed=True)
#         time.sleep(2)
#         marker_client2.send_update('marker', 3, detected=True)
#         time.sleep(2)
#         marker_client2.send_update('marker', 4, detected=True)

#         # Part 2: Waypoint Updates
#         if marker_client.is_waypoint_available(1):
#             marker_client.send_update('waypoint', 1, detected=True)
#             print("Heading to waypoint 1. Insert drone navigation/search code here.")
#             time.sleep(2)
#             print("Waypoint 1 completed.")
#             marker_client.send_update('waypoint', 1, detected=False)
#         else:
#             print("Unable to head to waypoint 1. Insert wait and check again is_waypoint_available.")
#             marker_client.send_update('status', status_message="Unable to head to waypoint 1")

#         if marker_client.is_waypoint_available(2):
#             marker_client.send_update('waypoint', 2, detected=True)
#             print("Heading to waypoint 2. Insert drone navigation/search code here.")
#             time.sleep(2)
#             print("Waypoint 2 completed.")
#             marker_client.send_update('waypoint', 2, detected=False)
#         else:
#             print("Unable to head to waypoint 2. Insert wait and check again is_waypoint_available.")
#             marker_client.send_update('status', status_message="Unable to head to waypoint 2")

# except KeyboardInterrupt:
#     print("User interrupt. Exiting.")
# except Exception as e:
#     print(f"Error: {e}. Exiting.")

# finally:
#     print("drone.end() here")
#     marker_client.send_update('status', status_message='Landed')

