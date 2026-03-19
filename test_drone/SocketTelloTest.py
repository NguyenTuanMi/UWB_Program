import socket
import threading
import time

# Raspberry Pi's Ethernet IP and Tello's command port
TELLO_IP = '192.168.10.1'
TELLO_PORT = 8889

# Local port to listen for responses
LOCAL_PORT = 8889

#Create UDP socket
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', LOCAL_PORT))  #Bind to all interfaces on LOCAL_PORT
def send_command(message):
    try:
        sock.sendto(message.encode(), (TELLO_IP, TELLO_PORT))
        print(f"Sent: {message}")
    except Exception as e:
        print(f"Failed to send {message}: {e}")

def receive_responses():
    #print("This thing is still Running")
    while True:
        try:
            response, ip = sock.recvfrom(1024)
            print("Beginning of Receiving Response")
            print(f"Received: {response.decode()}")
        except OSError as e:
            if e.errno == 9:  # Bad file descriptor
                # Socket was closed intentionally, exit silently
                print("Error 9")
                break
            else:
                print(f"Error receiving data: {e}")
                break

# Start the receiving thread
receive_thread = threading.Thread(target=receive_responses)
receive_thread.daemon = True
receive_thread.start()

# Define a sequence of commands with delays
commands = [
    ("command", 1),
    #("takeoff",5),
    ("motoron",5),
    ('motoroff',5),
    #("streamon", 5),
    ("battery?",1),
    ('takeoff', 10),
    ('forward 360', 10),
    ('cw 90', 5),
    ('forward 360', 10),
    ('cw 90', 5),
    ('forward 360', 10),
    ('cw 90', 5),
    ('forward 360', 10),
    ('cw 90', 5),
    #   ("cw 90",5),
    #("ccw 90",5),
    #("streamoff", 1),
    ("land",5)
]

# Send commands with delays
for cmd, delay in commands:
    send_command(cmd)
    time.sleep(delay)

print("Mission completed successfully!")

# Allow some time to receive all responses before closing
time.sleep(2)

# Close the socket gracefully
sock.close()
