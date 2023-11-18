#!/usr/bin/python3

"""
arm_control.py

Desc: Uses the CANbus interface to control arm position
Author: Ethan Cabelin
Date: 11/1/2023
"""

from cysar.msg import ArmPosition
from SparkCANLib.SparkCAN import SparkBus

# CAN IDs for Flipper Controllers
SHOULDER_RMOTOR = 0 # TODO: Retrieve from RevHardwareClient
SHOULDER_AMOTOR = 0 # TODO: Retrieve from RevHardwareClient
ELBOW_AMOTOR = 0 # TODO: Retrieve from RevHardwareClient
WRIST_RMOTOR = 0 # TODO: Retrieve from RevHardwareClient
WRIST_AMOTOR = 0 # TODO: Retrieve from RevHardwareClient
CLAW_CMOTOR = 0 # TODO: Retrieve from RevHardwareClient

INVERTED = -1

class Motor:
    """
    Class holding values for arm motors.
    """

    """
    Args:
        bus (SparkCANLib.SparkCAN.SparkBus): CANbus interface 
        id (int): CAN ID for the motor
    """
    def __init__(self, bus : SparkBus, id : int) -> None:
        self.controller = bus.init_controller(id)
        self.home = 0

    def set_home(self, motor : int) -> None:
        """
        Sets the current location of the motor as its home position.
        """
        self.home = self.controller.position

    def go_home(self, motor : int) -> None:
        """
        Sends a motor to their home position.
        """
        self.rotate_motor_position(self.home)

    def get_position(self):
        """
        Retrieves the current position of the flipper.
        """
        return self.controller.position

    def rotate_motor_position(self, position : int):
        """
        Rotates the flippers to a designated position relative to home.
        """
        self.controller.position_output(position + self.home)


class ArmControl():
    """
    Uses the CANbus interface to set the position of the arm motors.

    Args:
        bus (SparkCANLib.SparkCAN.SparkBus): CANbus interface
    """
    def __init__(self, bus : SparkBus) -> None:
        self.shoulder_RMotor = Motor(bus, SHOULDER_RMOTOR)
        self.shoulder_AMotor = Motor(bus, SHOULDER_AMOTOR)
        self.elbow_AMotor = Motor(bus, ELBOW_AMOTOR)
        self.wrist_RMotor = Motor(bus, WRIST_RMOTOR)
        self.wrist_AMotor = Motor(bus, WRIST_AMOTOR)
        self.claw_CMotor = Motor(bus, CLAW_CMOTOR)

    def set_positions(self, msg : ArmPosition) -> None:
        """
        Sets the position of all the arm motors based on the ROS values.
        
        Args:
            msg (ArmPosition): The values from ROS indicating the position of each motor
        """
        # Positive Position
        self.shoulder_RMotor.rotate_motor_position(msg.shoulder_rotatation)
        self.shoulder_AMotor.rotate_motor_position(msg.shoulder_angle)
        self.elbow_AMotor.rotate_motor_position(msg.elbow_angle)
        self.wrist_RMotor.rotate_motor_position(msg.wrist_rotation)
        self.wrist_AMotor.rotate_motor_position(msg.wrist_angle)
        self.claw_CMotor.rotate_motor_position(msg.claw_closure)
        
        