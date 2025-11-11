"""
Works 20 Feb
Run directly in terminal to open GUI for swarmserver.
"""

import threading, socket, json, time, logging, argparse
from typing import Dict, Set, Any, List, Literal
import tkinter as tk
from tkinter import ttk

class MarkerServer:
    def __init__(self, host='0.0.0.0', port=5005, show_waypoints_window=False):
        self.host = host
        self.port = port
        self.marker_timeout = 5
        self.waypoint_timeout = 10
        self.marker_status: Dict[str, Dict[str, Any]] = {}
        self.drone_status: Dict[str, Dict[str, Any]] = {}
        self.takeoff_waitlist = set()
        self.waypoints_status: Dict[str, Dict[str, Any]] = {}
        self.clients: Set[tuple] = set()
        self.lock = threading.Lock()
        self.last_updates: Dict[str, float] = {}
        self.takeoff_triggered = False
        self.show_waypoints_window = show_waypoints_window

        # Initialize sockets and logging as before
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.sock.bind((self.host, self.port))

        self.broadcast_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        self.broadcast_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        self.valid_ids = set(range(1, 9))

        logging.info(f"Swarm server started on {self.host}:{self.port}")

        # Initialize GUI
        self.root = tk.Tk()
        self.root.title("Marker and Drone Status Monitor")
        self.root.geometry("400x600")

        # Create a table to display marker statuses
        self.marker_tree = ttk.Treeview(self.root, columns=("Marker ID", "Detected", "Landed", "Drone ID"), show="headings")
        self.marker_tree.heading("Marker ID", text="Marker ID")
        self.marker_tree.heading("Detected", text="Detected")
        self.marker_tree.heading("Landed", text="Landed")
        self.marker_tree.heading("Drone ID", text="Drone ID")
        self.marker_tree.pack(fill=tk.BOTH, expand=True)

        # Create a table to display drone statuses
        self.drone_tree = ttk.Treeview(self.root, columns=("Drone ID", "Ready", "Waiting List", "Status"), show="headings")
        self.drone_tree.heading("Drone ID", text="Drone ID")
        self.drone_tree.heading("Ready", text="Ready")
        self.drone_tree.heading("Waiting List", text="Waiting List")
        self.drone_tree.heading("Status", text="Status")
        self.drone_tree.pack(fill=tk.BOTH, expand=True)

        # Adjust column width dynamically
        self.root.update_idletasks()
        self.adjust_column_widths()

        # Bind window resize event
        self.root.bind("<Configure>", lambda event: self.adjust_column_widths())

        # Add buttons for takeoff and landing
        self.takeoff_button = tk.Button(self.root, text="Takeoff Ready Drones", command=self.trigger_takeoff)
        self.takeoff_button.pack(fill=tk.X, pady=10)

        self.land_button = tk.Button(self.root, text="Land All Drones", command=self.trigger_land)
        self.land_button.pack(fill=tk.X, pady=10)

        # Initialize waypoints status window if enabled
        if self.show_waypoints_window:
            self.waypoints_root = tk.Toplevel(self.root)
            self.waypoints_root.title("Waypoints Status Monitor")
            self.waypoints_root.geometry("400x300")

            self.waypoints_tree = ttk.Treeview(self.waypoints_root, columns=("Waypoint ID", "Occupied", "Drone ID"), show="headings")
            self.waypoints_tree.heading("Waypoint ID", text="Waypoint ID")
            self.waypoints_tree.heading("Occupied", text="Occupied")
            self.waypoints_tree.heading("Drone ID", text="Drone ID")
            self.waypoints_tree.pack(fill=tk.BOTH, expand=True)

        # Start updating the GUI periodically
        self.update_gui()

    def trigger_takeoff(self):
        """
        Sends a takeoff signal to all drones that are ready.
        """
        ready_drones = [drone_id for drone_id, status in self.drone_status.items() if status.get("ready", False)]
        if ready_drones:
            self.takeoff_triggered = True     # resets countdown in wait_and_takeoff
            logging.info(f"Takeoff triggered by user. Drones currently ready: {ready_drones}")      # NOTE: logic and ready_drones in wait_and_takeoff
        else:
            logging.warning("No drones are ready for takeoff.")

    def trigger_land(self, send_repeat:int = 3):
        """
        Sends a land signal to all connected drones.
        """
        logging.info("Triggering landing...")
        land_message = json.dumps({"type": "land"}).encode()
        with self.lock:
            for client_addr in self.clients:
                for _ in range(send_repeat):
                    try:
                        self.broadcast_sock.sendto(land_message, client_addr)
                    except Exception as e:
                        logging.warning(f"Failed to send land signal to client {client_addr}: {e}")
        logging.info(f"Land signal sent to all drones {send_repeat} times for reliability. ")

    def check_timeouts(self):
        """Check for markers and waypoints that haven't been updated and clear their status.
        Also check if all valid markers have landed, and if so, reset all landed flags."""
        while True:
            try:
                current_time = time.time()
                status_changed = False
                
                with self.lock:
                    # Check for marker timeouts
                    for marker_id in list(self.marker_status.keys()):
                        last_update = self.last_updates.get(marker_id, 0)
                        if current_time - last_update > self.marker_timeout:
                            if marker_id in self.marker_status and self.marker_status[marker_id].get("detected", False):
                                logging.info(f"Clearing detected status for marker {marker_id} due to {self.marker_timeout}s timeout")
                                self.marker_status[marker_id]["detected"] = False
                                status_changed = True
                    
                    # Check for waypoint timeouts (1 minute timeout)
                    for waypoint_id in list(self.waypoints_status.keys()):
                        waypoint_data = self.waypoints_status[waypoint_id]
                        if waypoint_data.get("occupied", False):
                            last_occupied_time = waypoint_data.get("last_occupied_time", 0)
                            if current_time - last_occupied_time > self.waypoint_timeout:
                                logging.info(f"Releasing waypoint {waypoint_id} due to {self.waypoint_timeout}s timeout")
                                self.waypoints_status[waypoint_id]["occupied"] = False
                                status_changed = True
                    
                    # Check if all valid markers have landed
                    all_landed = True
                    valid_markers_exist = False
                    
                    for marker_id in self.valid_ids:
                        str_marker_id = str(marker_id)
                        if str_marker_id in self.marker_status:
                            valid_markers_exist = True
                            if not self.marker_status[str_marker_id].get("landed", False):
                                all_landed = False
                                break
                        else:  # not all valid IDs are in marker_status - not all have landed!
                            all_landed = False
                            break
                    
                    # Reset all landed flags if all valid markers have landed
                    if all_landed and valid_markers_exist:
                        logging.info(f"All valid markers {self.valid_ids} have landed. Resetting landed flags.")
                        for marker_id in self.marker_status:
                            if self.marker_status[marker_id].get("landed", False):
                                self.marker_status[marker_id]["landed"] = False
                                status_changed = True
                
                if status_changed:
                    self.broadcast_status('marker_status', self.marker_status)  # Broadcast updates to all clients
                    self.broadcast_status('waypoint_status', self.waypoints_status)  # Broadcast waypoint updates
                
                time.sleep(1)  # Check every second
            except Exception as e:
                logging.error(f"Error in check_timeouts: {e}")
                raise

    def update_marker_status(self, message:Dict):
        """Update marker status and timestamp"""
        with self.lock:
            try:
                marker_id = str(message["marker_id"])
                if marker_id != "-1" and marker_id is not None:
                    # Initialize marker status if not exists
                    if marker_id not in self.marker_status:
                        self.marker_status[marker_id] = {
                            "detected": False,
                            "landed": False
                        }

                    self.marker_status[marker_id]["drone_id"] = message.get("drone_id", 0)  # Registers which drone ID detected that marker
                    
                    # Update status
                    if "detected" in message:
                        self.marker_status[marker_id]["detected"] = message["detected"]
                        # Only update timestamp if marker is detected
                        if message["detected"]:
                            self.last_updates[marker_id] = time.time()
                    
                    if "landed" in message:
                        self.marker_status[marker_id]["landed"] = message["landed"]
                    
                    logging.debug(f"Updated marker {marker_id} status: {self.marker_status[marker_id]}")
            except Exception as e:
                logging.error(f"Error updating marker status: {e}")

        logging.info(f"Marker Statuses: {self.marker_status}")

    def update_waypoint_status(self, message: Dict):
        """NEW 6 MAR - Update waypoint occupied status and timestamp."""
        with self.lock:
            try:
                waypoint_id = str(message["marker_id"])
                if waypoint_id != "-1" and waypoint_id is not None:
                    # Initialize waypoint status if not exists
                    if waypoint_id not in self.waypoints_status:
                        self.waypoints_status[waypoint_id] = {
                            "occupied": False,
                            "last_occupied_time": 0  # Initialize last_occupied_time
                        }

                    self.waypoints_status[waypoint_id]["drone_id"] = message.get("drone_id", 0)  # Registers which drone ID detected that waypoint
                    
                    # Update status
                    if "detected" in message:
                        self.waypoints_status[waypoint_id]["occupied"] = message["detected"]
                        if message["detected"]:  # Update last_occupied_time only if waypoint is occupied
                            self.waypoints_status[waypoint_id]["last_occupied_time"] = time.time()
                    
                    logging.debug(f"Updated waypoint {waypoint_id} status: {self.waypoints_status[waypoint_id]}")
            except Exception as e:
                logging.error(f"Error updating waypoint status: {e}")

        logging.info(f"Waypoint Statuses: {self.waypoints_status}")

    def update_drone_status(self, message):
        """
        Updates drones' readiness status based on message received.
        """
        drone_id = message["drone_id"]

        if drone_id not in self.drone_status:
            self.drone_status[drone_id] = {"ready": False}

        # if "ready" in message:
        #     # self.drone_status[drone_id]["ready"] = message["ready"]
        #     # self.drone_status[drone_id]["status"] = message.get("status", "WOW") 
        #     self.drone_status[drone_id]["status"] = message['status'] 
        #     # self.drone_status[drone_id]["status"] = "WOW WHAT1"
        #     logging.debug(f"{drone_id} ready")

        if message["type"] == "takeoff_request":
            logging.debug(f"{drone_id} requesting takeoff. Message: {message}")
            self.drone_status[drone_id]["status"] = "WOW WHAT2"
            waiting_list = message["waiting_list"]
            status = message.get("status", None)
            self.register_ready_drone(drone_id, waiting_list, status)

        elif message["type"] == "status":
            logging.debug(f"{drone_id} status update. Message: {message}")
            self.drone_status[drone_id]["status"] = message["status"]

        logging.info(f"Drone Statuses: {self.drone_status}")

    def register_ready_drone(self, drone_id, waiting_list:List, status_str:str = None):
        """
        Registers a drone as ready and checks if all required drones are also ready.
        """
        with self.lock:
            self.drone_status[drone_id] = {
                "ready": True,
                "waiting_list": waiting_list,
                "status": status_str
            }
            all_waiting_drones = set(waiting_list) | {drone_id}

            logging.debug(f"all_waiting_drones: {all_waiting_drones}")

            # Check if we are the first drone to be ready in this group
            if not (set(waiting_list) & self.takeoff_waitlist):  # No intersection
                logging.info(f"Drone {drone_id} is first ready, starting countdown for {all_waiting_drones}")
                threading.Thread(target=self.wait_and_takeoff, args=(all_waiting_drones,)).start()
            else:
                logging.info(f"Drone {drone_id} is ready, but waiting for others.")

            self.takeoff_waitlist.update(all_waiting_drones)
            logging.debug(f"Current Waitlist: {self.takeoff_waitlist}")
    
    def handle_messages(self):
        prev_message = None
        while True:
            try:
                data, addr = self.sock.recvfrom(1024)
                try:
                    message:dict = json.loads(data.decode())
                    logging.debug(f"Received message from {addr}: {message}")

                    if message != prev_message:
                    
                        # Handle client registration message
                        if message.get("marker_id") == -1:
                            if addr not in self.clients:
                                with self.lock:
                                    self.clients.add(addr)
                                    logging.info(f"New client connected: {addr}")
                                self.broadcast_status_to_one(addr, 'marker_status', self.marker_status)
                        elif message.get("type") == 'takeoff_request' or message.get("type") == 'status':
                            self.update_drone_status(message)
                        elif message.get("type") == 'marker':
                            self.update_marker_status(message)
                        elif message.get("type") == 'waypoint':
                            self.update_waypoint_status(message)
                        else:
                            logging.warning("Invalid message format received. See handle_messages in swarmserverclient")
                            if addr not in self.clients:
                                with self.lock:
                                    self.clients.add(addr)
                                    logging.info(f"New client connected: {addr}")
                                self.broadcast_status_to_one(addr, "marker_status", self.marker_status)
                                self.broadcast_status_to_one(addr, "waypoint_status", self.waypoints_status)
                    
                    prev_message = message
                    self.broadcast_status('marker_status', self.marker_status)
                    self.broadcast_status('waypoint_status', self.waypoints_status)     # NEW NEW NEW 6 MAR
                        
                except json.JSONDecodeError:
                    logging.warning(f"Received invalid JSON from {addr}: {data}")
                    
            except Exception as e:
                logging.error(f"Error handling message: {e}")

    def broadcast_status_to_one(self, addr, message_type:Literal["marker_status", "waypoint_status"], message):
        try:
            message_to_send = {"type": message_type, "message": message}
            status_json = json.dumps(message_to_send).encode()
            self.broadcast_sock.sendto(status_json, addr)
        except Exception as e:
            logging.error(f"Error broadcasting to {addr}: {e}")

    def broadcast_status(self, message_type:Literal["marker_status", "waypoint_status"], message):
        try:
            with self.lock:
                message_to_send = {"type": message_type, "message": message}
                status_json = json.dumps(message_to_send).encode()
                
                dead_clients = set()
                for client_addr in self.clients:
                    try:
                        self.broadcast_sock.sendto(status_json, client_addr)
                    except Exception as e:
                        logging.warning(f"Failed to send to client {client_addr}: {e}")
                        dead_clients.add(client_addr)
                
                # Remove dead clients
                self.clients -= dead_clients
                if dead_clients:
                    logging.info(f"Removed dead clients: {dead_clients}")
        except Exception as e:
            logging.error(f"Error in broadcast_status: {e}")

    def run(self):
        try:
            threads = [
                threading.Thread(target=self.handle_messages, daemon=True),
                threading.Thread(target=self.check_timeouts, daemon=True)
            ]
            for thread in threads:
                thread.start()

            # Start the GUI main loop
            self.root.mainloop()

            # # Keep main thread alive
            # while True:
            #     time.sleep(2)
            #     logging.debug("GUI closed, but server is still running...")

            logging.info("Swarm server terminated.")
        except Exception as e:
            logging.error(f"Error in server run: {e}")

    ### SIMUL TAKEOFF FUNCTIONS

    def wait_and_takeoff(self, all_waiting_drones, takeoff_timeout=600, threshold=0.8):
        """
        NEW FXN 17 FEB
        Waits for all drones in the waiting list to be ready before sending a takeoff signal.
        2 Mar - does not close thread; cannot run twice
        """
        logging.debug("wait_and_takeoff thread started.")
        start_time = time.time()
        while time.time() - start_time < takeoff_timeout:
            status_copy = self.drone_status.copy()
            ready_drones_inwaitlist = [d for d in all_waiting_drones if status_copy.get(d, {}).get("ready", False)]     # THIS ASSUMES THE FIRST DRONE REGISTERED HAS THE CORRECT WAITLIST - other drones' waitlists are disregarded 
            ready_drones = [drone_id for drone_id, status in self.drone_status.items() if status.get("ready", False)]
            if set(ready_drones_inwaitlist) == set(all_waiting_drones):
                logging.info(f"All drones {ready_drones_inwaitlist} ready for takeoff!")
                self.send_takeoff_signal(ready_drones_inwaitlist)
                break
            elif self.takeoff_triggered:
                logging.info(f"User triggered takeoff. Drones {ready_drones} ready for takeoff!")
                self.send_takeoff_signal(ready_drones)
                break
            time.sleep(1)
            logging.debug(f"Waiting: {round((time.time() - start_time),0)}/{takeoff_timeout}s. Ready drones: {ready_drones}. Waitlist set by 1st ready drone: {all_waiting_drones}")

        if not self.takeoff_triggered:
            if len(ready_drones) / len(all_waiting_drones) >= threshold:
                logging.warning(f"{takeoff_timeout}s timeout reached! {len(ready_drones)}/{len(all_waiting_drones)} drones ready. Exceeded threshold of {threshold*100}%. Sending only these drones: {ready_drones}")
                self.send_takeoff_signal(ready_drones)
            else:
                logging.error(f"{takeoff_timeout}s timeout reached! {len(ready_drones)}/{len(all_waiting_drones)} drones ready. Takeoff aborted.")
        
        self.takeoff_triggered = False
        logging.debug("wait_and_takeoff thread ended.")

    def send_takeoff_signal(self, ready_drones:List, send_repeat: int = 3):
        """
        Sends takeoff signal to all ready drones.
        """
        if not ready_drones:
            logging.warning("No drones were ready for takeoff.")
            return

        takeoff_message = json.dumps({"type": "takeoff", "takeoff_list": ready_drones}).encode()
        with self.lock:
            for client_addr in self.clients:
                for _ in range(send_repeat):  # Send the message N times for reliability
                    try:
                        self.broadcast_sock.sendto(takeoff_message, client_addr)
                        time.sleep(0.01)  # Small 10ms delay to prevent flooding
                    except Exception as e:
                        logging.warning(f"Failed to send to client {client_addr}: {e}")

                    logging.debug(f"MarkerServer sent {takeoff_message} to {client_addr} (sent {send_repeat} times for reliability)")
    
    ### 20 FEB GUI FUNCTIONS

    def adjust_column_widths(self):
        """Adjust column widths dynamically based on window size."""
        total_width = self.root.winfo_width()
        
        dec1 = float(1/4)
        self.marker_tree.column("Marker ID", width=int(total_width * dec1))  # 25% of window width
        self.marker_tree.column("Detected", width=int(total_width * dec1))  
        self.marker_tree.column("Landed", width=int(total_width * dec1))    
        self.marker_tree.column("Drone ID", width=int(total_width * dec1))  

        dec2 = float(1/5)
        self.drone_tree.column("Drone ID", width=int(total_width * dec2/2))     
        self.drone_tree.column("Ready", width=int(total_width * dec2))       
        self.drone_tree.column("Waiting List", width=int(total_width * dec2))
        self.drone_tree.column("Status", width=int(total_width * dec2 * 2.5))

    def update_gui(self):
        """Update the GUI with the latest marker, drone, and waypoints statuses."""
        try:
            # Clear the current tables
            for row in self.marker_tree.get_children():
                self.marker_tree.delete(row)
            for row in self.drone_tree.get_children():
                self.drone_tree.delete(row)

            # Add rows for each marker status
            for marker_id, status in self.marker_status.items():
                if isinstance(status, dict):
                    detected = status.get("detected", False)
                    landed = status.get("landed", False)
                    drone_id = status.get("drone_id", 0)

                    detected_str = "✔ True" if detected else "❌ False"
                    landed_str = "✔ True" if landed else "❌ False"

                    self.marker_tree.insert("", "end", values=(marker_id, detected_str, landed_str, drone_id))

            # Add rows for each drone status
            for drone_id, status in self.drone_status.items():
                if isinstance(status, dict):
                    ready = status.get("ready", False)
                    waiting_list = status.get("waiting_list", [])
                    status_str = status.get("status", "no status")

                    ready_str = "✔ True" if ready else "❌ False"
                    waiting_list_str = ", ".join(map(str, waiting_list))

                    self.drone_tree.insert("", "end", values=(drone_id, ready_str, waiting_list_str, status_str))

            # Update waypoints status window if enabled
            if self.show_waypoints_window:
                for row in self.waypoints_tree.get_children():
                    self.waypoints_tree.delete(row)

                for waypoint_id, status in self.waypoints_status.items():
                    if isinstance(status, dict):
                        occupied = status.get("occupied", False)
                        drone_id = status.get("drone_id", 0)

                        occupied_str = "✔ True" if occupied else "❌ False"

                        self.waypoints_tree.insert("", "end", values=(waypoint_id, occupied_str, drone_id))

        except Exception as e:
            logging.error(f"Error updating GUI: {e}")

        # Schedule the next update (refresh every 500ms)
        self.root.after(500, self.update_gui)


class MarkerClient:
    def __init__(self, drone_id=0, server_port=5005, broadcast_ip="255.255.255.255", land_callback=None):
        self.drone_id = drone_id
        self.server_port = server_port
        self.broadcast_ip = broadcast_ip
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)  # Allow reuse of address
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)  # Enable broadcast
        self.sock.bind(("0.0.0.0", 0))  # Bind to any available port
        self.ready = False
        self.takeoff_signal = False
        self.land_signal = False
        self.land_callback = land_callback
        
        self.marker_status = {}  # Keeps a local copy of marker_status
        self.waypoint_status = {}  # Keeps a local copy of waypoint_status
        threading.Thread(target=self.receive_updates, daemon=True).start()

        self.send_update('marker', marker_id=-1)  # Send an initial message to register with the server
        logging.info(f"Marker client {drone_id} broadcasting on {self.broadcast_ip}:{self.server_port}")

    def client_takeoff_simul(self, drones_list:list, status_message:str = None):
        """
        This is the holding pattern that releases the Client once takeoff_signal is received from server.
        (caa 13 Mar) Must launch server BEFORE client can register for it to work!

        Args:
            drones_list: For manual clicking, use drones_list = [99]
        """
        self._send_takeoff_request(drones_list, status_message=status_message)
        
        while not self.takeoff_signal:      # This is a holding pattern, which releases upon takeoff_signal being set by the server.
            logging.debug(f"Tello {self.drone_id} waiting to take off. takeoff_signal: {self.takeoff_signal}")
            self._send_takeoff_request(drones_list, status_message=status_message)   # TBC 12 MAR - continue sending takeoff requests, just in case. (will it be too spammy?)
            time.sleep(0.5)  # Wait for takeoff command

        logging.info(f"Tello {self.drone_id} is taking off!")

    def _send_takeoff_request(self, waiting_list:list, status_message:str=None):
        """
        Notifies the server that this drone is ready for takeoff and is waiting for the specified drones.
        Internal method; Wouldn't usually call this yourself.
        """
        message = {
            "type": "takeoff_request",
            "drone_id": self.drone_id,
            "waiting_list": waiting_list,
            "ready": True,
            "status": status_message
        }
        message_json = json.dumps(message).encode()

        for _ in range(3):  # Send the message 5 times for reliability
            self.sock.sendto(message_json, (self.broadcast_ip, self.server_port))
            time.sleep(0.01)  # Small delay to prevent flooding

        logging.info(f"Drone {self.drone_id} is ready and waiting for {waiting_list} to takeoff together.")
    
    def send_update(self, 
                    update_type:Literal["status","marker","waypoint"], 
                    marker_id:int=None, detected:bool=None, landed:bool=None,
                    status_message:str='', 
                    send_repeat:int=3):
        """
        19 Feb - keep separate from _send_takeoff_request since it has an additional waiting_list argument

                message is a dictionary: 
                {"drone_id": 1, "update_type": "marker", "marker": 2, "detected": True}
                {"drone_id": 2, "update_type": "status", "status": "Centering"}

        26 Feb - can cause significant latency! changed 5 * 0.03s = 150ms to 3 * 0.01 = 30ms

        6 Mar - add new feature for ensuring non-conflicting waypoints execution
            send_update("waypoint", 1, detected = True) # ideally sends every 1s while searching
            # insert search here
            send_update("waypoint", 1, detected = False)

        """
        message:Dict = {"drone_id": self.drone_id}       # initializes the message with drone_id (client's class attribute)

        if update_type == "marker" and marker_id is not None:
            message["type"] = "marker"
            message["marker_id"] = marker_id
            if detected is not None:
                message["detected"] = detected
            if landed is not None:
                message["landed"] = landed

        elif update_type =="status":
            message["type"] = "status"
            message["status"] = status_message

        elif update_type =="waypoint":
            message["type"] = "waypoint"
            message["marker_id"] = marker_id
            message["detected"] = detected

        message_json = json.dumps(message).encode()
        for _ in range(send_repeat):  # Send the message N times for reliability
            self.sock.sendto(message_json, (self.broadcast_ip, self.server_port))
            time.sleep(0.01)  # Small 10ms delay to prevent flooding

        logging.debug(f"MarkerClient {self.drone_id} sent {message} (sent {send_repeat} times for reliability)")

    def receive_updates(self):  # Background thread
        while True:
            try:
                data, _ = self.sock.recvfrom(1024)
                message = json.loads(data.decode())

                if message.get("type") == "takeoff" and self.drone_id in message.get("takeoff_list"):
                    logging.info(f"Received takeoff signal for Tello {self.drone_id}")
                    self.takeoff_signal = True  # Set flag for DroneController         

                elif message.get("type") == "land":
                    logging.info(f"Received land signal for Tello {self.drone_id}")
                    # self.land_signal = True  # Set flag for DroneController to check  
                    if self.land_callback:  # Trigger the callback
                        self.land_callback() 
                
                elif message.get("type") == "marker_status":
                    logging.info(f"Received marker status")
                    self.marker_status = message.get("message", None)
                    logging.debug(f"Client's marker_status is now: {self.marker_status}")

                elif message.get("type") == "waypoint_status":
                    logging.info(f"Received waypoint status")
                    self.waypoint_status = message.get("message", None)
                    logging.debug(f"Client's waypoint_status is now: {self.waypoint_status}")

                else:
                    logging.error(f"MarkerClient's receive_update: SHOULD NOT BE HERE")
                

            except socket.timeout:  # Handle timeouts (if a timeout is set)
                pass
            except Exception as e:  # Catch any other exceptions
                logging.error(f"Error receiving updates: {e}")

    def is_marker_available(self, marker_id):
        marker_id = str(marker_id)  # Ensure it's a string to match dictionary keys
        marker_data:dict = self.marker_status.get(marker_id)
        if marker_data is None:
            return True  # Marker has never been seen before -> Available
        detected = marker_data.get("detected", False)   # False is the default value to return if "detected" key doesn't exist. If "detected": False, also returns false.
        landed = marker_data.get("landed", False)       # refer to TEST_markerstatusdict.py 
        return not (detected or landed)  # If either is True, it's NOT available. If no server, returns True by default (good for redundancy).
    
    def is_waypoint_available(self, waypoint_id):
        waypoint_id = str(waypoint_id)  # Ensure it's a string to match dictionary keys
        waypoint_data:dict = self.waypoint_status.get(waypoint_id, None)
        if waypoint_data is None:
            return True  # Marker has never been seen before -> Available
        occupied:bool = waypoint_data.get("occupied", False)   # False is the default value to return if "detected" key doesn't exist. If "detected": False, also returns false.
        return not occupied  # If occupied = True, return available = False. 
    
    def get_invalid_markers(self, markers_list: list) -> list:
        return [id for id in markers_list if not self.is_marker_available(id)]
    
    def cleanup(self):
        """Clean up the socket when the program exits."""
        if self.sock:
            self.sock.close()
            self.sock = None

    def __del__(self):
        """Destructor to ensure cleanup."""
        self.cleanup()

def main():
    parser = argparse.ArgumentParser(description="Run the Marker Server with optional waypoints status window.")
    parser.add_argument("-w", "--show-waypoints", action="store_true", help="Enable the waypoints status window")
    args = parser.parse_args()

    file_handler = logging.FileHandler("log_markerserver.log", mode='w')
    console_handler = logging.StreamHandler()
    formatter = logging.Formatter("%(levelname)s - %(asctime)s - %(message)s")
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)
    logging.basicConfig(level=logging.DEBUG, handlers=[file_handler, console_handler])

    server = MarkerServer(show_waypoints_window=args.show_waypoints)
    server.run()

if __name__ == "__main__": # Run directly to launch server!
    main()