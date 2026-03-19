import socket
import threading
import time

# Each drone: (TELLO_IP, TELLO_PORT, LOCAL_PORT)
DRONES = [
    ("192.168.0.105", 9005, 9005),
    ("192.168.0.106", 9006, 9006)
]

# Create sockets for each drone
sockets = []
for ip, tello_port, local_port in DRONES:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("", local_port))
    sockets.append((sock, ip, tello_port))

# Function to receive responses for one socket
def receive_responses(sock, drone_ip):
    while True:
        try:
            response, ip = sock.recvfrom(1024)
            print(f"[{drone_ip}] Received: {response.decode()}")
        except OSError as e:
            if e.errno == 9:  # Socket closed
                break
            else:
                print(f"[{drone_ip}] Error: {e}")
                break

# Start receiving threads for each drone
for sock, ip, port in sockets:
    t = threading.Thread(target=receive_responses, args=(sock, ip))
    t.daemon = True
    t.start()

# Define commands for all drones (same sequence for each)
commands = [
    ("command", 1),
    ("motoron", 5),
    ("motoroff", 5),
    ("takeoff", 10),
    ("cw 90", 5),
    ("land", 10),
]

# Send commands to all drones at once
for cmd, delay in commands:
    for sock, ip, port in sockets:
        try:
            sock.sendto(cmd.encode(), (ip, port))
            print(f"[{ip}] Sent: {cmd}")
        except Exception as e:
            print(f"[{ip}] Failed to send {cmd}: {e}")
    time.sleep(delay)

print("Mission completed successfully!")

time.sleep(2)  # Give time for last responses

# Close sockets
for sock, _, _ in sockets:
    sock.close()
