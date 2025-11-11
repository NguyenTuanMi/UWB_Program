# UWBViz - README

**Last Updated:** 19 Mar 2025

## Overview
GUI to display live position of all UWB tags in the arena. Tag data can then be used to record obstacles, waypoints etc., or simply track the motion of drones.

This code requires:
    1. `nlink_unpack_COMx_udp.exe` to be running on any other Windows device connected to the same network
        - This other device (Windows PC) is connected via USB to the LinkTrack Console and will transmit via UDP
        - Source file (for reference): main_udp.c
    2. `UWB_ReadUDP.py` custom library imported


## Usage
Controls
`R`- Start/Stop recording. Upon stopping recording, prompts user to save 1) Marked points 2) Waypoints separately.
    - The first save file contains ALL marked positions, including both obstacles and waypoints. 
    - The second save file contains only waypoints. This can be used in PPFLY2.
`U`- Toggle simulated tag #0 using mouse position
`Space` - Toggle UWB tag shadow tracing
`Enter` - Toggles pan lock
`M` - Load marked points (obstacles and/or waypoints) from json file in UWBViz/ directory
`L` - Load waypoints from json file in parent directory
`1~0` - Toggle UWB tag data to be used for recording [NOT WORKING CAA 19 MAR]. By default, records largest tag number available.

The following keys may be pressed only when recording in progress:
`Q` - Record waypoints
`Z` - Undo last marked item
`W` - Toggle wall recording start/stop
`P`/`V`/`D` - Mark pillars, victims and walls respectively

## Additional Notes

- **Note on UDP**: Using NTUSecure or RMTT/Tello WiFi is okay for transferring data within the same PC via UDP. However, it will not allow UDP transfer across devices. 
- **Running simulated UWB tag data** Run `UWB_SendUDP.py` (in UWB_Wrapper package) to simulate some drones' locations being broadcast via UDP, and visualize it on the GUI.