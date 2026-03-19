import socket

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.sendto(b"command", ("192.168.10.1", 8889))
sock.settimeout(5)
try:
    print(sock.recvfrom(1024))
except Exception as e:
    print("No response:", e)