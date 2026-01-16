# Params - README

**Last Updated:** 24 Feb 2025

## Overview

Each params file allows one script (in this case, UnknownArea_v2.main) to be run for multiple drones. It is loaded by load_params() under shared_utils.py. 

- `params.py`: Master file! Use this for testing on your own laptop's webcam without a drone
- `params0.py`: For flying via direct connection
- `paramsXX.py`: For flying via RPi (static IPs, port forwarding etc. configured manually prior)
