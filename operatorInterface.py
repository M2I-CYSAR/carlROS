import socket
import struct
import threading

DEADZONE = 0.05
class OI:

    def __init__(self):
        thread = threading.Thread(target=self.threadRoutine).start()
        self.LJoystickXAxisRaw = 0
        self.LJoystickYAxisRaw = 0
        self.AButtonRaw = 0
        self.BButtonRaw = 0
        self.data = 0

    def threadRoutine(self):
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
            dataRaw = conn.recv(1024)
            data = bytearray(dataRaw)
            self.LJoystickXAxisRaw = data[0]
            self.LJoystickYAxisRaw = data[1]
            self.AButtonRaw = data[2]
            self.BButtonRaw = data[3]

        # Close the connection
        conn.close()

    def getLeftJoystickXAxis(self):
        value = ((self.LJoystickXAxisRaw - 127.0) / 127.0)
        if abs(value) > DEADZONE:
            return value
        else:
            return 0

    def getLeftJoystickYAxis(self):
        value = ((self.LJoystickYAxisRaw - 127.0) / 127.0)
        if abs(value) > DEADZONE:
            return value
        else:
            return 0

    def getAButtonPressed(self):
        return self.AButtonRaw

    def getBButtonPressed(self):
        return self.BButtonRaw

