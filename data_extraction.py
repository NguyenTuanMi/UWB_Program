import socket

UDP_PORT = 5000

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', UDP_PORT))  # Listen on all interfaces

print("Listening for UWB data on UDP port", UDP_PORT)
while True:
    data, addr = sock.recvfrom(2048)
    line = data.decode().strip()
    print("Received:", line)
    # You can further parse the CSV line:
    # fields = line.split(',')
    # id, role, x, y, z, d0, d1, d2, d3, d4, d5, d6, d7 = fields