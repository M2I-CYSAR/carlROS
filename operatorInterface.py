import socket
import struct

# Define the IP address and port number to listen on
ip_address = "0.0.0.0"
port = 4143

# Create a TCP socket
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# Bind the socket to the IP address and port
s.bind((ip_address, port))

# Listen for incoming connections
s.listen()

# Accept a connection
conn, addr = s.accept()

while True:
    # Receive data
    data = conn.recv(4)
    LJoystickXAxis, LJoystickYAxis, AButton, BButton = struct.unpack('bbbb', data)
    print(f"XAxis: {LJoystickXAxis}, YAxis: {LJoystickYAxis}, A: {AButton}, B: {BButton}")

# Close the connection
conn.close()
