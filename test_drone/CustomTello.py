from djitellopy import Tello
import time
# Define configuration constants
NETWORK_CONFIG = {
'host': '192.168.0.105',
'control_port': 9005,
'state_port': 8005,
'video_port': 11105
}
class CustomTello(Tello):
    def __init__(self, network_config):
        # Store custom configuration; Treats RPi as the drone. All communication is with the RPi only
        self.TELLO_IP = network_config['host']
        self.CONTROL_UDP_PORT = network_config['control_port']
        self.STATE_UDP_PORT = network_config['state_port']
        self.VS_UDP_PORT = network_config['video_port']
        Tello.STATE_UDP_PORT = self.STATE_UDP_PORT
        Tello.CONTROL_UDP_PORT = self.CONTROL_UDP_PORT
        # Call parent's init with our custom host
        super().__init__(self.TELLO_IP)
        # Override the connection parameters
        self.address = (self.TELLO_IP, self.CONTROL_UDP_PORT)
        # Override video port
        self.vs_udp_port = self.VS_UDP_PORT
    
def main():
# Use the centralized network configuration
    controller = CustomTello(NETWORK_CONFIG)
    controller.connect()
    time.sleep(5)
    print(f"Battery Level: {controller.get_battery()}%")
    controller.streamon()
    time.sleep(60)
    controller.end()
if __name__ == "__main__":
    main()