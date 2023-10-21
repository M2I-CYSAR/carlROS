#!/usr/bin/python3

"""
flipper_control.py

Desc: Uses the CANbus interface to set the position of the flippers
Author: Isaac Denning
Date: 10/21/23
"""

from cysar.msg import FlipperPosition

# CAN IDs for Flipper Controllers
FLF = 21
FRF = 22
BLF = 23
BRF = 24

class Flipper:
    """
    Class for controlling individual flippers.

    Args:
        bus (?): CANbus interface 
        id (int): CAN ID for the flipper
    """
    def __init__(self, bus, id : int) -> None: #TODO
        self.controller = bus.init_controller(id)
        self.home = 0
        self.set_home()

    def set_home(self) -> None:
        """
        Sets the current location of the flippers as their home position.
        """
        self.home = self.controller.position

    def go_home(self) -> None:
        """
        Sends the flippers to their home position.
        """
        self.controller.position_output(self.home)

    def get_position(self): #TODO
        """
        Retrieves the current position of the flipper.
        """
        return self.controller.position

    def rotate_flipper_position(self, position):
        """
        Rotates the flippers to a designated position relative to home.
        """
        self.controller.position_output(position + self.home)


class FlipperControl():
    """
    Uses the CANbus interface to set the position of the flippers.

    Args:
        bus (?): CANbus interface
    """
    def __init__(self, bus) -> None: #TODO
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
        self.FLFlipper.rotate_flipper_position(msg.front_left)
        self.FRFlipper.rotate_flipper_position(msg.front_right)
        self.BLFlipper.rotate_flipper_position(msg.back_left)
        self.BRFlipper.rotate_flipper_position(msg.back_right)
