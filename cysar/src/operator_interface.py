#!/usr/bin/python3

"""
operator_inerface.py

Desc: Connection between base station and carl
Author: Isaac Denning
Date: 10/21/23
"""

import rclpy
from rclpy.node import Node
from cysar.msg import Joystick
import socket

class OperatorInterface(Node):
    """
    Creates connection between base station and ROS.
    """
    def __init__(self) -> None:
        super().__init__('Operator_Interface')
        self.joystick = Joystick()
        self.joystick_publisher = self.create_publisher(Joystick, 'joystick', 10)

        # Define the IP address and port number to listen on
        ip_address = "192.168.1.101"
        port = 4143

        # Create a TCP socket
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # Bind the socket to the IP address and port
        s.bind((ip_address, port))

        # Listen for incoming connections
        s.listen()

        # Accept a connection
        self.conn, addr = s.accept()

        # Startup the ROS publisher
        self.talker()

    def talker(self) -> None:
        """
        Loop for retrieving joystick inputs and publishing them to ROS.
        """
        while True:
            # Receive data
            dataRaw = self.conn.recv(1664)
            data = bytearray(dataRaw)
            self.joystick.stick_left_x = (data[0] - 127) / 127
            self.joystick.stick_left_y = (data[1] - 127) / 127
            self.joystick.stick_right_x = (data[2] - 127) / 127
            self.joystick.stick_right_y = (data[3] - 127) / 127
            self.joystick.button_a = data[4] == 1
            self.joystick.button_b = data[5] == 1
            self.joystick.button_x = data[6] == 1
            self.joystick.button_y = data[7] == 1
            self.joystick.button_start = data[8] == 1
            self.joystick.bumper_left = data[9] == 1
            self.joystick.bumper_right = data[10] == 1
            self.joystick.trigger_left = data[11] / 254
            self.joystick.trigger_right = data[12] /254
            
            # Publish data to ROS
            self.joystick_publisher.publish(self.joystick)

        # Close the connection
        conn.close()


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    operator_interface = OperatorInterface()

    # Run the node
    rclpy.spin(operator_interface)

    # Destroy it when done
    operator_interface.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()