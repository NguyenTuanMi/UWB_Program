# PPGUI - Drone Path Planner
Last updated: 4 Mar
## Overview

This module is a Drone Path Planner designed for UAV lab testing and field applications. It provides a graphical interface for planning waypoints, marking obstacles, and visualizing flight paths. The software is built using Pygame for rendering and Tkinter for user input handling.

- **NOTE**: As of 19 Mar 2025, most features in PPGUI have been ported over to UWBViz, as waypoints are now marked directly with UWB tags, rather than through mouse inputs.

## Features

- **Grid System**: Displays a 1m x 1m scaled grid for reference.
- **Waypoint Marking**: Allows users to click and set waypoints.
- **Obstacle and Structure Marking**: Supports marking walls, victims, dangers, and pillars.
- **JSON Waypoint Storage**: Saves and loads waypoints in JSON format.
- **Customizable Takeoff Zone**: Highlights the drone takeoff area.
- **Orientation & Flight Info Display**: Shows key instructions and guidance for drone heading.

## Installation

1. Ensure Python is installed (Python 3.7 or higher recommended).
2. Install required dependencies using:

   ```bash
   pip install pygame

## Usage
Controls
`S`- Save and close
`O` - Enable orthogonal mode
`Z` - Undo last waypoint
`R` - Reset all waypoints and obstacles
`M` - Load/clear marked points (obstacles) from json file in UWBViz/ directory
`Mouse Click` - Add waypoints