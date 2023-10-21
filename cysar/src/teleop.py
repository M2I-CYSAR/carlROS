#!/usr/bin/python3

"""
teleop.py

Desc: Main functionality of overall program. Takes ROS joystick values and 
        decides what to publish for drive, flipper, and arm. 
Author: Isaac Denning
Date: 10/21/23
"""

import rclpy
from rclpy.node import Node
from cysar.msg import Joystick, DriveTrain, FlipperPosition, ArmPosition
import numpy as np

DEADZONE = 0.05
MAX_SPEED = 1
FLIPPER_SENSITIVITY = 0.03

class Teleop(Node):
    """
    Main functionality of overall program. Takes ROS joystick values and 
    decides what to publish for drive, flipper, and arm.
    """
    def __init__(self) -> None:
        super().__init__('Teleop')
        self.joystick = Joystick()
        self.drive_train = DriveTrain()
        self.flipper_position = FlipperPosition()
        self.flipper_position.front_left = 0.0
        self.flipper_position.front_right = 0.0
        self.flipper_position.back_left = 0.0
        self.flipper_position.back_right = 0.0
        self.drive_train_publisher = self.create_publisher(DriveTrain, 'drive_train', 10)
        self.flipper_position_publisher = self.create_publisher(FlipperPosition, 'flipper_position', 10)
        self.joystick_subscription = self.create_subscription(Joystick, 'joystick', self.listener, 10)

    def listener(self, msg : Joystick) -> None:
        """
        Takes joystick values and publish movement out.

        Args:
            msg (Joystick): The ROS joystick values recieved by operator_interface,py 
        """
        self.joystick = msg
        self.talker()

    def talker(self) -> None:
        """
        Uses saved joystick values to publish movement out.
        """
        self.drive_train_update()
        self.flipper_position_update()

    def drive_train_update(self) -> None:
        """
        Uses saved joystick values to publish new the drive train velocies.
        """
        # Unnecessarily complex equation for finding velocity of motors based on joystick position
        x = self.joystick.stick_left_x if abs(self.joystick.stick_left_x) > DEADZONE else 0
        y = self.joystick.stick_left_y if abs(self.joystick.stick_left_y) > DEADZONE else 0
        r2o2 = np.sqrt(2) / 2
        dist = np.sqrt(x*x + y*y)
        normX = x / dist if dist != 0 else 0
        normY = y / dist if dist != 0 else 0
        percent_left = np.minimum(1, np.arccos(np.minimum(abs(normX * r2o2 + normY * r2o2), 1)) / np.pi * 2 * dist) * np.sign(y - x)
        percent_right = np.minimum(1, np.arccos(np.minimum(1, abs(normX * -r2o2 + normY * r2o2))) / np.pi * 2 * dist) * np.sign(y + x)

        # Clamp them to the max speed
        self.drive_train.front_left = MAX_SPEED * percent_left
        self.drive_train.back_left = MAX_SPEED * percent_left
        self.drive_train.front_right = MAX_SPEED * percent_right
        self.drive_train.back_right = MAX_SPEED * percent_right

        # Publish drive_trian
        self.drive_train_publisher.publish(self.drive_train)

    def flipper_position_update(self) -> None:
        """
        Uses saved joystick values to publish new the flipper positions.
        """

        if abs(self.joystick.stick_right_y) > DEADZONE:
            # Move flipper values based on joystick
            self.flipper_position.front_left += float(self.joystick.stick_right_y * self.joystick.bumper_left * FLIPPER_SENSITIVITY)
            self.flipper_position.front_right += float(self.joystick.stick_right_y * self.joystick.bumper_right * FLIPPER_SENSITIVITY)
            self.flipper_position.back_left += float(self.joystick.stick_right_y * (self.joystick.trigger_left > DEADZONE) * FLIPPER_SENSITIVITY)
            self.flipper_position.back_right += float(self.joystick.stick_right_y * (self.joystick.trigger_right > DEADZONE) * FLIPPER_SENSITIVITY)

            # Clip them if above or below min max of -1 and 1
            self.flipper_position.front_left = float(np.clip(self.flipper_position.front_left, -1, 1))
            self.flipper_position.front_right = float(np.clip(self.flipper_position.front_right, -1, 1))
            self.flipper_position.back_left = float(np.clip(self.flipper_position.back_left, -1, 1))
            self.flipper_position.back_right = float(np.clip(self.flipper_position.back_right, -1, 1))

        # Send to zero if start is pressed
        if self.joystick.button_start:
            self.flipper_position.front_left = 0.0
            self.flipper_position.front_right = 0.0
            self.flipper_position.back_left = 0.0
            self.flipper_position.back_right = 0.0

        # Publish flipper_position
        self.flipper_position_publisher.publish(self.flipper_position)

def msg_data(msg : any) -> str:
    """
    Takes any msg type are returns a string showing all its members and their values.
    Very useful for debuging.
    """
    result = ""

    result += f'({type(msg)})\n'

    if not hasattr(msg, "get_fields_and_field_types"):
        return result

    fields = msg.get_fields_and_field_types()
    # Iterate through the dictionary and print the values
    for field, field_type in fields.items():
        if hasattr(msg, field):
            result += f'{field}: {getattr(msg, field)}\n'
    return result

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    teleop = Teleop()

    # Run the node
    rclpy.spin(teleop)

    # Destroy it when done
    teleop.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()