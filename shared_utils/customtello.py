from djitellopy import Tello
import threading
import socket
import djitellopy.tello

class CustomTello(Tello):
    def __init__(self, network_config):
        self.TELLO_IP = network_config['host']
        self.LOCAL_CONTROL_PORT = network_config['control_port'] # e.g., 38065
        self.STATE_UDP_PORT = network_config['state_port']       # e.g., 8895
        self.VS_UDP_PORT = network_config['video_port']          # e.g., 11115
        
        self.TELLO_CONTROL_PORT = 8889 # The hardware port the drone always listens to
        self.retries = 10

        # -------------------------------------------------------------------
        # 1. THE HIJACK: Prevent base class from spawning default threads
        # -------------------------------------------------------------------
        djitellopy.tello.threads_initialized = True

        # -------------------------------------------------------------------
        # 2. COMMAND RECEIVER: Bind locally, start thread
        # -------------------------------------------------------------------
        djitellopy.tello.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        djitellopy.tello.client_socket.bind(("", self.LOCAL_CONTROL_PORT))

        response_receiver_thread = threading.Thread(target=Tello.udp_response_receiver)
        response_receiver_thread.daemon = True
        response_receiver_thread.start()

        # -------------------------------------------------------------------
        # 3. STATE RECEIVER: Override global target, start thread
        # -------------------------------------------------------------------
        # The base method `udp_state_receiver` looks at `Tello.STATE_UDP_PORT`.
        # We overwrite it here before starting the thread.
        Tello.STATE_UDP_PORT = self.STATE_UDP_PORT

        state_receiver_thread = threading.Thread(target=Tello.udp_state_receiver)
        state_receiver_thread.daemon = True
        state_receiver_thread.start()

        # -------------------------------------------------------------------
        # 4. INITIALIZE PARENT
        # -------------------------------------------------------------------
        super().__init__(host=self.TELLO_IP, retry_count=self.retries)

        # -------------------------------------------------------------------
        # 5. ROUTING OVERRIDES
        # -------------------------------------------------------------------
        # Point outgoing commands to the drone's 8889 port
        self.address = (self.TELLO_IP, self.TELLO_CONTROL_PORT)
        
        # Tell the video receiver (PyAV) which local port to listen on
        self.vs_udp_port = self.VS_UDP_PORT

        # Custom attributes
        self.using_down_vision = False
        self.frame = None 
        self.frame_lock = threading.Lock()
        self.is_running = False