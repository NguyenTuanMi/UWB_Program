# SwarmServer User Guide
**Updated: 6 Mar**

## Overview
`swarmserverclient.py` is an all-in-one module containing both `MarkerServer` and `MarkerClient` classes.  
- A single `MarkerServer` must be running per network.  
- The server **must be started before** drone initialization to handle takeoff requests.  

🔹 **See `example.py` for a simple implementation using a real drone.**
🔹 **See `example_nodrone.py` for a more thorough implementation without a drone.**
🔹 **Ensure `swarmserverclient.py` is running before starting both scripts!**

## Installation & Usage

1. **Import `swarmserverclient.py`** and initialize `MarkerClient` in your code.
2. **Start the server**: Run `swarmserverclient.py` before takeoff. Use the **-w** flag to enable the waypoints window.
   ```bash
   python swarmserverclient.py -w
   ```
3. **Run the drone script, then trigger takeoff from GUI**:
   - Use `client_takeoff_simul(waiting_list)` to initiate takeoff once all drones in the list are ready. Place it after drone.connect() and before drone.takeoff().
   - If manual triggering is needed, use `waiting_list = [99]`. For auto triggering, use `waiting list = [1,2,3,4]` to trigger takeoff immediately once drone IDs 1,2,3,4 are ready. 
   - You can also add a status message (usually for battery level) to display while drone is waiting to takeoff. E.g.`client_takeoff_simul(waiting_list, 'drone.get_battery()')`

## Important Notes

⚠️ **Placement of client_takeoff_simul**  
- Since it is only a hold-and-release function, place it **after drone.connect()** and **before drone.takeoff()**.

⚠️ **Landing function limitation**  
- The **"Land all drones"** function requires additional implementation in your code.  
- It sends a `land_signal`, which must be processed by your main loop or a separate thread to trigger `drone.end()`.  

⚠️ **Takeoff function limitation**  
- The server only works for triggering each drone ID **once**. If the same drone reconnects to a continually running server, it will not be able to trigger takeoff (caa 6 Mar; TBC why)