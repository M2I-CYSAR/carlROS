#!/usr/bin/python3

"""
flipper_control.py

Desc: Uses the CANbus interface to set the position of the flippers
Author: Isaac Denning
Date: 10/21/23
"""

from cysar.msg import FlipperPosition
from SparkCANLib.SparkCAN import SparkBus

# CAN IDs for Flipper Controllers
FLF = 21
FRF = 22
BLF = 23
BRF = 24

INVERTED = -1

class Flipper:
    """
    Class for controlling individual flippers.

    Args:
        bus (SparkCANLib.SparkCAN.SparkBus): CANbus interface 
        id (int): CAN ID for the flipper
    """
    def __init__(self, bus : SparkBus, id : int) -> None:
        self.controller = bus.init_controller(id)
        self.home = 0

    def set_home(self) -> None:
        """
        Sets the current location of the flippers as their home position.
        """
        self.home = self.controller.position

    def go_home(self) -> None:
        """
        Sends the flippers to their home position.
        """

        print(self.controller.position)

        # self.controller.position_output(self.home)
        self.current_position = self.controller.position
        if self.current_position > 0:
            while self.current_position > 0:
                self.current_position -= 1
                self.controller.position_output(self.current_position)
                self.sleep(0.2)
                print(self.current_position)
        elif self.current_position < 0:
            while self.current_position < 0:
                self.current_position += 1
                self.controller.position_output(self.current_position)
                self.sleep(0.2)
                print(self.current_position)
        self.controller.position_output(self.home)



    def get_position(self):
        """
        Retrieves the current position of the flipper.
        """
        return self.controller.position

    def rotate_flipper_position(self, position : float):
        """
        Rotates the flippers to a designated position relative to home.

        Args:
            position (float): Position to set the motor to
        """
        self.controller.position_output(position + self.home)


class FlipperControl():
    """
    Uses the CANbus interface to set the position of the flippers.

    Args:
        bus (SparkCANLib.SparkCAN.SparkBus): CANbus interface
    """
    def __init__(self, bus : SparkBus) -> None:
        self.FLFlipper = Flipper(bus, FLF)
        self.FRFlipper = Flipper(bus, FRF)
        self.BLFlipper = Flipper(bus, BLF)
        self.BRFlipper = Flipper(bus, BRF)

    def set_positions(self, msg : FlipperPosition) -> None:
        """
        Sets the position of all the flippers based on the ROS values.

        Args:
            msg (FlipperPosition): The values from ROS indicating the position of each flipper
        """
        self.FLFlipper.rotate_flipper_position(msg.front_left * INVERTED)
        self.FRFlipper.rotate_flipper_position(msg.front_right)
        self.BLFlipper.rotate_flipper_position(msg.back_left)
        self.BRFlipper.rotate_flipper_position(msg.back_right * INVERTED)
