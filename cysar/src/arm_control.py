#!/usr/bin/python3

"""
arm_control.py

Desc: Uses the CANbus interface to control arm position
Author: Ethan Cabelin
Date: 11/1/2023
"""

from cysar.msg import ArmPosition
from SparkCANLib.SparkCAN import SparkBus
import RPi.GPIO as GPIO

# CAN IDs for Flipper Controllers
SHOULDER_RMOTOR = 31
SHOULDER_AMOTOR = 32
ELBOW_AMOTOR = 33
WRIST_RMOTOR = 34
WRIST_AMOTOR = 35
CLAW_CLOSE_PIN = 19
CLAW_OPEN_PIN = 21

INVERTED = -1

class Motor:
    """
    Class holding values for arm motors.

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

    def rotate_motor_position(self, position : float):
        """
        Rotates the flippers to a designated position relative to home.
        """
        self.controller.position_output(position + self.home)

    def rotate_motor_percent(self, velocity : float):
        self.controller.percent_output(velocity)

class LinearActuator:
    """
    Class for controlling claw linear actuator.

    Args:
        extendPin (int): The GPIO pin which, when high, extends the actuator
        contractPin (int): The GPIO pin which, when high, contracts the actuator
    """
    def __init__(self, extendPin : int, contractPin : int) -> None:
        self.extendPin = extendPin
        self.contractPin = contractPin
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(self.extendPin, GPIO.OUT, initial=GPIO.LOW)
        GPIO.setup(self.contractPin, GPIO.OUT, initial=GPIO.LOW)

    def extend(self) -> None:
        """
        Sets the GPIO pins to extend the actuator.
        """
        GPIO.output(self.extendPin, GPIO.HIGH)
        GPIO.output(self.contractPin, GPIO.LOW)

    def contract(self) -> None:
        """
        Sets the GPIO pins to contract the actuator.
        """
        GPIO.output(self.extendPin, GPIO.LOW)
        GPIO.output(self.contractPin, GPIO.HIGH)

    def cleanup(self) -> None:
        """
        Cleans up the GPIO after use.
        """
        GPIO.cleanup()

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
        self.claw_Actuator = LinearActuator(CLAW_OPEN_PIN, CLAW_CLOSE_PIN)

    def set_positions(self, msg : ArmPosition) -> None:
        """
        Sets the position of all the arm motors based on the ROS values.
        
        Args:
            msg (ArmPosition): The values from ROS indicating the position of each motor
        """
        self.shoulder_RMotor.rotate_motor_position(msg.shoulder_rotatation)
        self.shoulder_AMotor.rotate_motor_position(msg.shoulder_angle)
        self.elbow_AMotor.rotate_motor_position(msg.elbow_angle)
        self.wrist_RMotor.rotate_motor_percent(INVERTED * msg.wrist_rotation_velocity)
        self.wrist_AMotor.rotate_motor_percent(msg.wrist_angle_velocity)
        if msg.claw_closing:
            self.claw_Actuator.contract()
        else:
            self.claw_Actuator.extend()
        
        