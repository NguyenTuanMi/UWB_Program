"""
This is to fly through AP Mode (NOTE: Previous control_port + 20).
"""


LAPTOP_ONLY:bool = False   # indicate LAPTOP_ONLY = True to use MockTello() and laptop webcam instead
NO_FLY:bool = False        # indicate NO_FLY = True to connect to the drone, but ensure it doesn't fly while the video feed still appears
IMSHOW:bool = True        # indicate IMSHOW = False to disable video stream display (program still works!)

PI_ID:int = 13   # 0 for LAPTOP_ONLY
UWBTAG_ID:int = 0

# WAYPOINTS_JSON = "wpfinal_USAGAB13_straightonly.json"
WAYPOINTS_JSON = "waypoint_13.json"
# WAYPOINTS_JSON = "UWBViz/waypoints_3m.json"

FLIGHT_HEIGHT_WAYPOINTS:int = 70
FLIGHT_HEIGHT_SEARCH:int = 120

TAKEOFF_HOVER_DELAY:int = 0
PRE_TAKEOFF_DELAY:int = 180


### LESS COMMONLY TOUCHED PARAMS ###

import numpy as np

# CAMERA MATRIX + DIST COEFF (480P)
CAMERA_MATRIX: np.ndarray = np.array([[472.23497738,   0.,         314.44428497],
                                      [  0.,         471.39817591, 226.70646477],
                                      [  0.,           0.,           1.        ]])

DIST_COEFF: np.ndarray = np.array([[ 0.03218703,  0.17998838,  0.00076689, -0.0075065,  -0.28000622]])

LOGGING_CONFIG = {
    'filename': f"log_Pi{PI_ID if PI_ID else ''}.log",
    'level': "DEBUG",  # Change this to "INFO", "WARNING", etc.
    'format': "%(levelname)s - %(name)s - %(asctime)s - %(message)s",
    'default_logger_name': "DroneController"
}

EXTRA_HEIGHT = 0   # cm; if victim is higher than ground level (especially if detecting vertical face) 

def get_network_config(pi_id: int):
    """
    Returns network configuration based on pi_id.
    Args:
        pi_id: None for direct WiFi connection, or an integer (e.g., 2, 12) for RPi connection
    Returns:
        dict: Network configuration with host and port settings
    """
    print("Pi ID is", pi_id)
    if pi_id == 0:
        return {
            'host': '192.168.10.1',     # if connected directly through WiFi
            'control_port': 8889,
            'state_port': 8890,
            'video_port': 11111    
        }
    else:
        return {
            'host': f'192.168.0.{100+pi_id}',
            # 'control_port': 9000 + pi_id,
            'control_port': 9000 + pi_id,
            'state_port': 8000 + pi_id,
            'video_port': 11100 + pi_id
        }   

NETWORK_CONFIG = get_network_config(PI_ID)    # Function defined at bottom of file

if LAPTOP_ONLY:     # just in case
    NO_FLY = True
    get_network_config(0)     

    DRONE_1_CONFIG = {

}