#!/usr/bin/python3

"""
can_control.py

Desc: Sets up the can bus and calls the controllers for the corrisponding parts 
        when (drive, flippper, arm) when ROS data is recieved.
Author: Isaac Denning
Date: 10/21/23
"""

import rclpy
from rclpy.node import Node
from cysar.msg import FlipperPosition, DriveTrain, ArmPosition
from SparkCANLib import SparkController, SparkCAN
from drive_control import DriveControl
from flipper_control import FlipperControl
#TODO: Import arm_control 
from arm_control import ArmControl

class CanControl(Node):
    """
    Sets up the can bus and calls the controllers for the corresponding parts 
        when (drive, flippper, arm) when ROS data is recieved.
    """
    def __init__(self) -> None:
        super().__init__('can_control')
        self.bus = SparkCAN.SparkBus(channel="can0", bustype='socketcan', bitrate=1000000)
        self.flipper_control = FlipperControl(self.bus)
        self.drive_control = DriveControl(self.bus)
        self.arm_control = ArmControl(self.bus)
        self.flipper_subscription = self.create_subscription(FlipperPosition, 'flipper_position', self.flipper_listener, 10)
        self.drive_train_subscription = self.create_subscription(DriveTrain, 'drive_train', self.drive_listener, 10)
        self.arm_subscription = self.create_subscription(ArmPosition, 'arm_position', self.arm_listener, 10)

    def flipper_listener(self, msg : FlipperPosition) -> None:
        """
        Called whenever new flipper position data is recieved from ROS.
        """
        self.flipper_control.set_positions(msg)

    def drive_listener(self, msg : DriveTrain) -> None:
        """
        Called whenever new drive train data is recieved from ROS.
        """
        self.drive_control.set_velocity(msg)

    def arm_listener(self, msg : ArmPosition) -> None:
        """
        Called whenever new arm position data is received from ROS
        """
        self.arm_control.set_positions(msg)

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    can_control = CanControl()

    # Run the node
    rclpy.spin(can_control)

    # Destroy it when done
    can_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()