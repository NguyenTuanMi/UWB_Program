"""
Import Hierarchy: constants -> config -> utils -> main
EVERYTHING HERE CAN BE ADJUSTED!
"""
BGPIC = None       # for empty screen!
BGPIC = "UWB_Manipulation/resources/field2025_old_overlay.png"
# BGPIC = "resources/blank.jpg"

# Scaling Coeffs
# ACTUAL_HEIGHT, ACTUAL_WIDTH = 2000, 2000  # cm of actual space (competition; nanyang audi)
# ACTUAL_HEIGHT, ACTUAL_WIDTH = 1500, 1500  # cm of actual space (arc lab)
# ACTUAL_HEIGHT, ACTUAL_WIDTH = 600, 600  # cm of actual space (uav lab)
ACTUAL_HEIGHT, ACTUAL_WIDTH = 440, 440 #cm of actual space (uav lab smaller area)
WIDTH_HEIGHT_RATIO = ACTUAL_WIDTH/ACTUAL_HEIGHT

SCREEN_HEIGHT = 600     # pixels on screen
SCREEN_WIDTH = int(SCREEN_HEIGHT*WIDTH_HEIGHT_RATIO)

MAP_SIZE_COEFF = ACTUAL_WIDTH / SCREEN_WIDTH  # cm per pixel; ASSUMES 1:1 ratio
print(f"cm per px = {MAP_SIZE_COEFF}")

WAYPOINTS_JSON_DEFAULT = "waypoint20x20"
MARKEDPOINTS_JSON_DEFAULT = "uwb_trace"

INITIAL_HEADING = 180       # can adjust

ORT_MODE = False
INT_MODE = False
INTERMEDIATE_DIST = 50

# Colors
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)
ORANGE = (255, 165, 0)
GREY = (200, 200, 200)

# # Positions for 20x20
# ENDPOS = (400, 0)
# STARTPOS_1_CM = (1000, 200) # ORIGIN @ bottom left
# STARTPOS_2_CM = (1000, 800)
# STARTPOS_3_CM = (1800, 800)
# STARTPOS_4_CM = (1800, 200)

# Positions for 6x6
ENDPOS = (400, 0)
# STARTPOS_1_CM = (1000, 200) # ORIGIN @ bottom left
# STARTPOS_2_CM = (1000, 800)
# STARTPOS_3_CM = (1800, 800)
# STARTPOS_4_CM = (1800, 200)

STARTPOS_1_CM = (50, 50) # ORIGIN @ bottom left
STARTPOS_2_CM = (50, 100)
STARTPOS_3_CM = (100, 100)
STARTPOS_4_CM = (100, 50)

#    This is to convert origin from top-left of screen (pygame default) to bottom-left (UWB)
STARTPOS_1 = (STARTPOS_1_CM[0]/MAP_SIZE_COEFF, (ACTUAL_HEIGHT-STARTPOS_1_CM[1])/MAP_SIZE_COEFF) 
STARTPOS_2 = (STARTPOS_2_CM[0]/MAP_SIZE_COEFF, (ACTUAL_HEIGHT-STARTPOS_2_CM[1])/MAP_SIZE_COEFF)
STARTPOS_3 = (STARTPOS_3_CM[0]/MAP_SIZE_COEFF, (ACTUAL_HEIGHT-STARTPOS_3_CM[1])/MAP_SIZE_COEFF)
STARTPOS_4 = (STARTPOS_4_CM[0]/MAP_SIZE_COEFF, (ACTUAL_HEIGHT-STARTPOS_4_CM[1])/MAP_SIZE_COEFF)