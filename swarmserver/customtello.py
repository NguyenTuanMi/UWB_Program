"""
18 Feb - only imported by shared_utils.dronecontroller2; no need to import into main.py!
"""

import cv2
import numpy as np
from djitellopy import Tello
import time
import logging  # in decreasing log level: debug > info > warning > error > critical
import threading
import random
import socket
import djitellopy.tello as _tello_module

#from UWB_Wrapper.UWB_SendUDP import UWBPublisher

class CustomTello(Tello):
    
    RESPONSE_TIMEOUT = 7    # Alternative: override globally here (default: 7s)
    TAKEOFF_TIMEOUT = 10    # (default: 20s)]
    
    
    # def __init__(self,
    #              host=TELLO_IP,
    #              retry_count=RETRY_COUNT,
    #              vs_udp=VS_UDP_PORT):

    #     global threads_initialized, client_socket, drones

    #     self.address = (host, Tello.CONTROL_UDP_PORT)
    #     self.stream_on = False
    #     self.retry_count = retry_count
    #     self.last_received_command_timestamp = time.time()
    #     self.last_rc_control_timestamp = time.time()

    #     if not threads_initialized:
    #         # Run Tello command responses UDP receiver on background
    #         client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    #         client_socket.bind(("", Tello.CONTROL_UDP_PORT))
    #         response_receiver_thread = Thread(target=Tello.udp_response_receiver)
    #         response_receiver_thread.daemon = True
    #         response_receiver_thread.start()

    #         # Run state UDP receiver on background
    #         state_receiver_thread = Thread(target=Tello.udp_state_receiver)
    #         state_receiver_thread.daemon = True
    #         state_receiver_thread.start()

    #         threads_initialized = True

    #     drones[host] = {'responses': [], 'state': {}}

    #     self.LOGGER.info("Tello instance was initialized. Host: '{}'. Port: '{}'.".format(host, Tello.CONTROL_UDP_PORT))

    #     self.vs_udp_port = vs_udp

    def __init__(self, network_config):
        host         = network_config['host']
        control_port = network_config['control_port']
        state_port   = network_config['state_port']
        video_port   = network_config['video_port']

        # Prevent super().__init__ from creating the shared global socket bound
        # to 8889 — that would fail on the 2nd drone and corrupt all responses.
        _tello_module.threads_initialized = True

        # super().__init__ sets self.address, self.retry_count, timestamps, and
        # registers drones[host]. It skips socket/thread creation because we
        # set threads_initialized = True above.
        super().__init__(host=host, retry_count=10, vs_udp=video_port)

        # Correct the command destination (super sets it to port 8889 via class attr)
        self.address    = (host, control_port)
        self.vs_udp_port = video_port

        # Per-instance command socket bound to a random OS-assigned port (not 8889)
        self._cmd_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._cmd_socket.bind(('0.0.0.0', 0))   # 0 → OS picks a free port
        self._cmd_socket.settimeout(10.0)

        # Register this drone in djitellopy's response dict and point the
        # module-level client_socket at ours so any un-overridden djitellopy
        # internals that reference it don't NameError.
        _tello_module.drones[host]    = {'responses': [], 'state': {}}
        _tello_module.client_socket   = self._cmd_socket

        # Per-instance receiver threads
        threading.Thread(target=self._response_receiver, daemon=True,
                         name=f'tello_resp_{host}').start()
        threading.Thread(target=self._state_receiver, args=(host, state_port),
                         daemon=True, name=f'tello_state_{host}').start()

        # Custom attributes
        self.using_down_vision = False
        self.frame      = None
        self.frame_lock = threading.Lock()
        self.is_running = False

    # ------------------------------------------------------------------
    # Per-instance socket receivers
    # ------------------------------------------------------------------

    def _response_receiver(self):
        host = self.address[0]
        while True:
            try:
                data, _ = self._cmd_socket.recvfrom(1024)
                _tello_module.drones[host]['responses'].append(data)
            except socket.timeout:
                continue
            except Exception as e:
                self.LOGGER.error(f"[{host}] response_receiver error: {e}")
                break

    def _state_receiver(self, host: str, state_port: int):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('0.0.0.0', state_port))
        sock.settimeout(5.0)
        while True:
            try:
                data, addr = sock.recvfrom(1024)
                if addr[0] != host:
                    continue   # ignore stray packets from other drones
                _tello_module.drones[host]['state'] = Tello.parse_state(data.decode('ASCII'))
            except socket.timeout:
                continue
            except Exception as e:
                self.LOGGER.error(f"[{host}] state_receiver error: {e}")
                break

    # ------------------------------------------------------------------
    # Route commands through per-instance socket (not the global one)
    # ------------------------------------------------------------------

    def send_command_with_return(self, command: str, timeout: int = 1) -> str:
        diff = time.time() - self.last_received_command_timestamp
        if diff < self.TIME_BTW_COMMANDS:
            time.sleep(diff)
        self.LOGGER.info(f"[{self.address[0]}] Send: '{command}'")
        timestamp = time.time()
        self._cmd_socket.sendto(command.encode('utf-8'), self.address)
        responses = self.get_own_udp_object()['responses']
        while not responses:
            if time.time() - timestamp > timeout:
                msg = f"No response to '{command}' after {timeout}s"
                self.LOGGER.warning(msg)
                return msg
            time.sleep(0.1)
        self.last_received_command_timestamp = time.time()
        try:
            return responses.pop(0).decode('utf-8').rstrip('\r\n')
        except UnicodeDecodeError:
            return 'response decode error'

    def send_command_without_return(self, command: str):
        # send_rc_control calls this, so RC commands are routed correctly too
        self._cmd_socket.sendto(command.encode('utf-8'), self.address)

    def get_udp_video_address(self) -> str:
        return f'udp://@0.0.0.0:{self.vs_udp_port}'
    
    def get_frame(self):
        with self.frame_lock:
            return self.frame.copy() if self.frame is not None else None

    def set_frame(self, frame):
        with self.frame_lock:
            self.frame = frame


    def rc_pulse(self, lr=0, fb=0, ud=0, dur=0.3):
        self.send_rc_control(int(lr), int(fb), int(ud), 0)
        time.sleep(float(dur))
        self.send_rc_control(0, 0, 0, 0)
    
    def go_to_height(self, height: int) -> None:
        """
        Custom Tello function to go to exact height, works caa 6 Feb (Gab)
        :param: height: Desired height in cm
        :return: None
        """
        current_height_tof = self.get_distance_tof()
        current_height_ = self.get_height()
        diff = height - current_height_tof
        
        if diff >= 20:
            self.move_up(diff)  # diff is already positive
        elif diff <= -20:  # Changed from abs(diff) <= -20
            self.move_down(abs(diff))  # need abs() since move_down needs positive value
        elif abs(diff) < 5:    # Close enough, accepted
            print(f"Height diff {diff}cm is less than 5cm. Moving on.")
        elif abs(diff) < 20:    # Resets, recurses
            self.move_up(30)
            self.go_to_height(height)
                
        final_height_tof = self.get_distance_tof()
        print(f"go_to_height {height}cm complete. Final height {final_height_tof}cm.")
            
    def go_to_height_PID(self, target_height: int, timeout: float = 10.0) -> bool:      # TESTING 5 FEB
        """
        Control drone height using PID control until target height is reached within tolerance
        
        Args:
            target_height: Desired height in cm
            timeout: Maximum time to attempt height control in seconds
        Returns:
            bool: True if target height was achieved, False if timeout occurred
        """
        # PID constants - these may need tuning
        Kp = 0.3  # Proportional gain
        Ki = 0.1  # Integral gain
        Kd = 0.1  # Derivative gain
        
        # Control parameters
        tolerance = 5  # Acceptable error in cm
        min_speed = 20  # Minimum speed for drone movement
        max_speed = 100  # Maximum speed for drone movement
        dt = 0.1  # Time step in seconds
        
        # Initialize PID variables
        integral = 0
        prev_error = 0
        start_time = time.time()
        
        while True:
            # Check timeout
            if time.time() - start_time > timeout:
                logging.warning(f"Height control timeout after {timeout} seconds")
                return False
                
            # Get current height
            current_height = self.get_distance_tof()
            error = target_height - current_height
            
            # Check if we're within tolerance
            if abs(error) < tolerance:
                logging.info(f"Target height achieved. Current: {current_height}cm, Target: {target_height}cm")
                self.send_rc_control(0,0,0,0)
                return True
                
            # Calculate PID terms
            integral += error * dt
            derivative = (error - prev_error) / dt
            
            # Calculate control output
            output = (Kp * error) + (Ki * integral) + (Kd * derivative)
            
            # Convert output to speed command
            speed = int(abs(output))
            speed = max(min_speed, min(speed, max_speed))  # Clamp speed between min and max
            logging.debug(f"Output: {output}, Speed: {speed}")

            # Apply control
            try:
                if output > 0:
                    if speed >= min_speed:
                        self.send_rc_control(0,0,speed,0)
                else:
                    if speed >= min_speed:
                        self.send_rc_control(0,0,-speed,0)
            except Exception as e:
                logging.error(f"Error during height adjustment: {e}")
                return False
                
            # Update previous error
            prev_error = error
            
            # Small delay to prevent overwhelming the drone
            time.sleep(dt)
            
            # Log progress
            logging.info(f"Current height: {current_height}cm, Error: {error}cm, Speed: {speed}")
    
    def get_ext_tof(self) -> int:  
        """Get ToF sensor reading"""
        start_time = time.time()
        response = self.send_read_command("EXT tof?")
        end_time = time.time()
        duration = end_time - start_time
        try:
            logging.debug(f"get_ext_tof response time: {duration:.2f}s")    # normally under 0.5-1s, default timeout is 7s
            return int(response.split()[1])
        except ValueError or IndexError as e:    # IndexError if using threading and the response is not as intended
            logging.debug(f"EXT ToF response time: {duration:.2f}s")
            logging.debug(f"get_ext_tof raises error: {e}. Returning 8888.")    # 13 Feb seldom actually reaches here without threading?
            return 8888
        except:
            logging.debug(f"EXT ToF response time: {duration:.2f}s")
            logging.debug(f"get_ext_tof raises other error: {e}. Returning 8888.")    # 13 Feb seldom actually reaches here without threading?
            return 8888

class MockFrameReader:
    def __init__(self, stream):
        self.stream = stream
        self.frame = None
        self.running = True  # Flag to control the thread
        self.lock = threading.Lock()  # Ensure thread-safe updates

        # Start a background thread to constantly update the frame
        self.thread = threading.Thread(target=self._update_frame, daemon=True)
        self.thread.start()

    def _update_frame(self):
        """Continuously update the latest frame in a separate thread."""
        while self.running:
            ret, frame = self.stream.read()
            if ret:
                with self.lock:  # Ensure thread-safe frame updates
                    self.frame = frame
            else:
                logging.warning("Failed to read frame from webcam.")
            time.sleep(0.03)  # ~30 FPS simulation

    def stop(self):
        """Stop the frame update thread."""
        self.running = False
        self.thread.join()

class MockTello:
    def __init__(self, uwb_sim=False, uwb_ip='127.0.0.1', uwb_port=5000, tag_id=0):
        self.stream = None  # Video capture object
        self.stream_on = False  # Stream state
        self.yaw = 0  # Initial yaw value
        self.height = 100  # Initial height in cm
        self.uwb_sim = uwb_sim  # Whether to simulate UWB data
        self.uwb_publisher = None  # UWB publisher for simulating position

        if uwb_sim:
            # Initialize UWB publisher
            self.uwb_publisher = UWBPublisher(ip=uwb_ip, port=uwb_port, tag_id=tag_id)
            self.uwb_publisher.start_publishing()  # Start publishing UWB data

    def connect(self):
        print("Mock: Drone connected.")

    def get_battery(self):
        print("Mock: Battery at 100%")
        return 100

    def set_mission_pad_detection_direction(self, direction):
        print(f"Mock: Set mission pad detection direction to {direction}")

    def takeoff(self):
        print("Mock: Taking off...")
        if self.uwb_sim:
            self.uwb_publisher.update_takeoff(self.height)  # Simulate takeoff

    def rotate_clockwise(self, angle):
        self.yaw = (self.yaw + angle) % 360
        if self.yaw > 180:
            self.yaw -= 360  # Ensures yaw stays in the range [-180, 180]
        print(f"Mock: Rotating clockwise by {angle} degrees. New yaw: {self.yaw} degrees.")
        if self.uwb_sim:
            self.uwb_publisher.update_rotate(angle)  # Update yaw in UWB publisher

    def rotate_counter_clockwise(self, angle):
        self.yaw = (self.yaw - angle) % 360
        if self.yaw > 180:
            self.yaw -= 360  # Ensures yaw stays in the range [-180, 180]
        print(f"Mock: Rotating counter-clockwise by {angle} degrees. New yaw: {self.yaw} degrees.")
        if self.uwb_sim:
            self.uwb_publisher.update_rotate(-angle)  # Update yaw in UWB publisher

    def move_forward(self, distance):
        print(f"Mock: Moving forward {distance} cm.")
        if self.uwb_sim:
            self.uwb_publisher.update_move_forward(distance)  # Simulate forward movement

    def move_right(self, distance):
        print(f"Mock: Moving right {distance} cm.")
        # if self.uwb_sim:
        #     self.uwb_publisher.update_move_forward(distance)  # Simulate forward movement

    def get_mission_pad_id(self):
        print("Mock: No mission pad found.")
        return -1  # Simulates no pad found

    def go_to_height(self, height: int):
        print(f"Mock: Moving up to {height} cm.")
        self.height = height
        if self.uwb_sim:
            self.uwb_publisher.update_takeoff(height)  # Update height in UWB publisher

    def go_to_height_PID(self, height: int):
        print(f"Mock: Moving up with PID to {height} cm.")
        self.height = height
        if self.uwb_sim:
            self.uwb_publisher.update_takeoff(height)  # Update height in UWB publisher

    def send_rc_control(self, x, y, z, yaw):
        print(f"Mock: Received rc control commands. x: {x}, y: {y}, z: {z}, yaw: {yaw}")
        self.yaw += yaw / 5  # Adjust the yaw based on the rc control command
        self.yaw = (self.yaw + 180) % 360 - 180  # Normalize yaw to [-180, 180]
        if self.uwb_sim:
            self.uwb_publisher.update_rc_control(x, y, z, yaw)  # Update velocity and yaw in UWB publisher

    def get_yaw(self) -> int:
        print(f"Mock: Current Yaw = {self.yaw}")
        return self.yaw

    def get_height(self) -> int:
        print(f"Mock: Current Height = {self.height}")
        return self.height

    def get_distance_tof(self) -> int:
        dist = random.randint(50, 100)  # Mock value for distance
        print(f"Mock: Downward ToF = {dist}cm")
        return dist

    def get_ext_tof(self, simulate_delay: bool = False) -> int:
        """
        :args:
        - simulate_delay:bool = Simulates real-life delay of up to 7 seconds
        """
        rand_num = np.random.beta(2, 8)  # Generates a number between 0 and 1
        delay = 0.3 + rand_num * (7 - 0.3)  # Scale to range [0.3, 7]
        ext_dist = random.randint(700, 1200)  # Simulate external ToF measurement
        
        if not simulate_delay:
            delay = 0

        if delay > 5:  # Simulate invalid reading
            delay = 7
            ext_dist = 8888

        if ext_dist > 700:  # Simulates no reading (i.e. clear of obstacles)
            ext_dist = 8191
        elif ext_dist > 1150:  # Simulate invalid reading
            ext_dist = 8888

        print(f"Mock: Ext ToF = {ext_dist}mm. Simulated response time {delay:.1f}s")
        time.sleep(delay)
        return ext_dist

    def streamon(self):
        """Start video stream from laptop webcam."""
        self.stream = cv2.VideoCapture(0)  # Open default camera (0)
        if not self.stream.isOpened():
            raise Exception("Could not open webcam")
        self.stream_on = True
        print("MockTello: Webcam stream started.")

    def streamoff(self):
        """Stop video stream."""
        if hasattr(self, "frame_reader"):  # Stop the frame reading thread
            self.frame_reader.stop()
        if self.stream is not None:
            self.stream.release()
        self.stream_on = False
        print("MockTello: Webcam stream stopped.")

    def get_frame_read(self):
        """Return a frame reader object that mimics Tello's BackgroundFrameRead."""
        if not self.stream_on or self.stream is None:
            raise Exception("Stream is not started. Call streamon() first.")

        self.frame_reader = MockFrameReader(self.stream)  # Start the frame reader
        return self.frame_reader  # Return an object with `.frame`

    @property
    def is_flying(self):
        return True

    def land(self):
        print("Mock: Landing...")
        if self.uwb_sim:
            self.uwb_publisher.update_land()  # Simulate landing

    def end(self):
        print("Mock: Landing and Ending...")
        if self.uwb_sim:
            self.uwb_publisher.stop_publishing()  # Stop publishing UWB data

    def __del__(self):
        """Ensure the webcam is released when the object is deleted."""
        self.streamoff()
        if self.uwb_sim:
            self.uwb_publisher.stop_publishing()  # Stop publishing UWB data