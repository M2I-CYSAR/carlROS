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
shoulderRMotor = 0 # TODO: Retrieve from RevHardwareClient
shoulderAMotor = 0 # TODO: Retrieve from RevHardwareClient
elbowAMotor = 0 # TODO: Retrieve from RevHardwareClient
wristRMotor = 0 # TODO: Retrieve from RevHardwareClient
wristAMotor = 0 # TODO: Retrieve from RevHardwareClient
clawAMotor = 0 # TODO: Retrieve from RevHardwareClient

INVERTED = -1

class arm:
    """
    Class holding values for arm motors.
    """

    """
    Args:
        bus (SparkCANLib.SparkCAN.SparkBus): CANbus interface 
    """
    def __init__(self, bus : SparkBus) -> None: # No need for an int initialization since we won't be calling different arms. 
        # // Think of Implementing Seperate Classes for these Nodules
        # Shoulder Nodules
        self.homeSR = 0
        self.shoulderRotation = self.bus.init_controller(shoulderRMotor) # Base Shoulder 360 Degrees 
        # 
        self.homeSA = 0
        self.shoulderAngle = self.bus.init_controller(shoulderAMotor) # Base Shoulder Pivot
        # Elbow Nodule 
        self.homeEA = 0
        self.elbowAngle = self.bus.init_controller(elbowAMotor) # Elbow Belt Drive
        # Wrist Nodules 
        self.homeWR = 0
        self.wristRotation = self.bus.init_controller(wristRMotor) # Wrist Rotation 360 Degrees
        self.homeWA = 0
        self.wristAngle = self.bus.init_controller(wristAMotor) # Review This Variable / Will the claw hava a pivoting wrist?
        # Claw Nodule
        self.homeCA = 0
        self.clawA = self.bus.init_controller(clawAMotor) 
        # An Array of ALL Motors
        # 0 = Shoulder Rotate
        # 1 = Shoulder Pivot
        # 2 = Elbow Pivot
        # 3 = Wrist Rotate
        # 4 = Wrist Pivot
        # 5 = Claw Closure
        self.ArmVals = [[self.shoulderRotation, self.homeSR], [self.shoulderAngle, self.homeSA], [self.elbowAngle, self.homeEA], [self.wristRotation, self.homeWR] , [self.wristAngle, self.homeWA], [self.clawA, self.homeCA]]

    #TODO: Implement Independent Home Function For Each Motor
    def set_home(self, motor : int) -> None:
        """
        Sets the current location of the flippers as their home position.
        Note: ArmVals[motor][0] = self.controller 
        """
        self.ArmVals[motor][1] = self.ArmVals[motor][0].position

    #TODO: Implement Independent Home Function For Each Motor
    def go_home(self, int : motor) -> None:
        """
        Sends a motor to their home position.

        """
        print(self.ArmVals[motor][0].position)

        # self.controller.position_output(self.home)
        # TODO: Consider how we want to adjust this function for different axis

        self.controller = self.ArmsVals[motor][0]
        self.home = self.ArmVals[motor][1]
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



    #TODO: Implement Independent Home Function For Each Motor
    def get_position(self, int : motor):
        """
        Retrieves the current position of the flipper.
        """
        return self.Armvals[motor].position

    def rotate_motor_position(self, position):
        """
        Rotates the flippers to a designated position relative to home.
        """
        self.controller = self.ArmsVals[motor][0]
        self.home = self.ArmVals[motor][1]

        self.controller.position_output(position + self.home)


class ArmControl():
    """
    Uses the CANbus interface to set the position of the arm motors.

    Args:
        bus (SparkCANLib.SparkCAN.SparkBus): CANbus interface
    """
    def __init__(self, bus : SparkBus) -> None:
        self.CySARM = arm(bus)

    def set_positions(self, msg : ArmPosition) -> None:
        """
        Sets the position of all the arm motors based on the ROS values.
        # 0 = Shoulder Rotate - int32 shoulder_rotatation
        # 1 = Shoulder Pivot - int32 shoulder_angle
        # 2 = Elbow Pivot - int32 elbow_angle
        # 3 = Wrist Rotate - int32 wrist_rotation
        # 4 = Wrist Pivot - int32 wrist_angle
        # 5 = Claw Closure - int32 claw_closure
        Args:
            msg (ArmPosition): The values from ROS indicating the position of each flipper
        """
        # Positive Position
        self.CySARM.rotate_motor_position(msg.shoulder_rotatation, 0)
        self.CySARM.rotate_motor_position(msg.shoulder_angle, 1)
        self.CySARM.rotate_motor_position(msg.elbow_angle, 2)
        self.CySARM.rotate_motor_position(msg.wrist_rotation, 3)
        self.CySARM.rotate_motor_position(msg.wrist_angle, 4)
        self.CySARM.rotate_motor_position(msg.claw_closure, 5)

        # Negative Position
        self.CySARM.rotate_motor_position(msg.shoulder_rotatation * INVERTED, 0)
        self.CySARM.rotate_motor_position(msg.shoulder_angle * INVERTED, 1)
        self.CySARM.rotate_motor_position(msg.elbow_angle * INVERTED, 2)
        self.CySARM.rotate_motor_position(msg.wrist_rotation* INVERTED, 3)
        self.CySARM.rotate_motor_position(msg.wrist_angle * INVERTED, 4)
        self.CySARM.rotate_motor_position(msg.claw_closure * INVERTED, 5)
        
        