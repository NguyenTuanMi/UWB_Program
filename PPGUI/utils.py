import pygame
import json
import math
from constants import *

def get_dist_btw_pos_px(pos0, pos1):
    """
    Get distance between 2 mouse positions (in px).
    Returns both cm and pixels.
    """
    x = abs(pos0[0] - pos1[0])
    y = abs(pos0[1] - pos1[1])
    dist_px = math.hypot(x, y)
    dist_cm = dist_px * MAP_SIZE_COEFF
    return int(dist_cm), int(dist_px)

def get_angle_btw_line(pos0, pos1, posref):
    """
    Get signed angle between two lines with respect to 'posref' using the dot product.
    CCW = +ve, CW = -ve
    """
    ax = posref[0] - pos0[0]
    ay = posref[1] - pos0[1]
    bx = posref[0] - pos1[0]
    by = posref[1] - pos1[1]
    
    _dot = (ax * bx) + (ay * by)
    _magA = math.sqrt(ax**2 + ay**2)
    _magB = math.sqrt(bx**2 + by**2)
    
    if _magA * _magB == 0:
        return 0

    rad = math.acos(_dot / (_magA * _magB))
    angle = (rad * 180) / math.pi
    
    # Compute the cross product to determine the sign of the angle
    cross_product = ax * by - ay * bx
    if cross_product < 0:
        angle = -angle

    print(f"Angle is {angle}")
    return int(angle)

def get_yaw_angle(pos0, pos1, posref):
    """
    Calculate the change in heading required to rotate from the direction vector 'pos0 -> posref'
    to align with the direction vector 'posref -> pos1'.
    """
    # Define the vectors from the points
    v1x, v1y = posref[0] - pos0[0], posref[1] - pos0[1]
    v2x, v2y = pos1[0] - posref[0], pos1[1] - posref[1]

    # Calculate the dot product and magnitudes of v1 and v2
    dot_product = v1x * v2x + v1y * v2y
    mag_v1 = math.sqrt(v1x**2 + v1y**2)
    mag_v2 = math.sqrt(v2x**2 + v2y**2)

    # Ensure there's no division by zero if any vector has zero magnitude
    if mag_v1 * mag_v2 == 0:
        return 0
    cos_theta = dot_product / (mag_v1 * mag_v2)
    cos_theta = max(-1.0, min(1.0, cos_theta))  # Ensure the value is within [-1, 1]; rounding errors 1.000002 sometimes

    # Calculate the angle's magnitude (in radians) using the dot product
    angle_rad = math.acos(cos_theta)
    angle_deg = math.degrees(angle_rad)

    # Use the cross product to determine the sign (direction) of the angle
    cross_product = v1x * v2y - v1y * v2x
    if cross_product > 0:
        angle_deg = -angle_deg  # Clockwise rotation (negative angle)

    # print(f"Heading change needed: {angle_deg} degrees")
    return angle_deg


# CHECKER FUNCTIONS

def print_json_waypoints(filename):
    """
    :param filename: Exclude .json extension (e.g. 'pathplanningGUI/waypoint20x20')
    Checker function to review generated json.
    """
    with open(f'{filename}.json', 'r') as f:
        data = json.load(f)
        
    # Print all waypoints
    for i, wp in enumerate(data['wp']):
        print(f"\nWaypoint {i+1}:")
        print(f"Position: ({wp['position_cm']['x']}cm, {wp['position_cm']['y']}cm)")
        print(f"Distance to next: {wp['dist_cm']}cm")
        print(f"Turn angle: {wp['angle_deg']}°")
