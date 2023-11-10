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

class Teleop(Node):
    """
    Main functionality of overall program. Takes ROS joystick values and 
    decides what to publish for drive, flipper, and arm.
    """
    def __init__(self) -> None:
        super().__init__('Teleop')
        # Variables
        self.joystick = Joystick()
        self.drive_train = DriveTrain()
        self.flipper_position = FlipperPosition()
        self.arm_position = ArmPosition()

        # Publisher/Subscribers
        self.drive_train_publisher = self.create_publisher(DriveTrain, 'drive_train', 10)
        self.flipper_position_publisher = self.create_publisher(FlipperPosition, 'flipper_position', 10)
        self.arm_position_publisher = self.create_publisher(ArmPosition, 'arm_positin', 10)
        self.joystick_subscription = self.create_subscription(Joystick, 'joystick', self.listener, 10)

        # Paramerters   
        self.declare_parameter('deadzone', '0.05')
        self.declare_parameter('max_speed', '1.0')
        self.declare_parameter('flipper_sensitivity', '1.0')
        self.declare_parameter('flipper_min', '-14.0')
        self.declare_parameter('flipper_max', '14.0')
        self.deadzone = self.get_parameter('deadzone').get_parameter_value().double_value
        self.max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self.flipper_sensitivity = self.get_parameter('flipper_sensitivity').get_parameter_value().double_value
        self.flipper_min = self.get_parameter('flipper_min').get_parameter_value().double_value
        self.flipper_max = self.get_parameter('flipper_max').get_parameter_value().double_value
        self.get_logger().info(f"""
deadzone: {self.deadzone}, 
max_speed: {self.max_speed}, 
flipper_sensitivity: {self.flipper_sensitivity}, 
flipper_min: {self.flipper_max}, 
flipper_max: {self.flipper_min}\n')
""")
        
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
        self.arm_position_update()
        self.get_logger().info(msg_data(self.drive_train))
        self.get_logger().info(msg_data(self.flipper_position))
        self.get_logger().info(msg_data(self.flipper_position))

    def drive_train_update(self) -> None:
        """
        Uses saved joystick values to publish new the drive train velocies.
        """
        # Unnecessarily complex equation for finding velocity of motors based on joystick position
        x = self.joystick.stick_left_x if abs(self.joystick.stick_left_x) > self.deadzone else 0
        y = self.joystick.stick_left_y if abs(self.joystick.stick_left_y) > self.deadzone else 0
        r2o2 = np.sqrt(2) / 2
        dist = np.sqrt(x*x + y*y)
        normX = x / dist if dist != 0 else 0
        normY = y / dist if dist != 0 else 0
        percent_left = np.minimum(1, np.arccos(np.minimum(abs(normX * r2o2 + normY * r2o2), 1)) / np.pi * 2 * dist) * np.sign(y - x)
        percent_right = np.minimum(1, np.arccos(np.minimum(1, abs(normX * -r2o2 + normY * r2o2))) / np.pi * 2 * dist) * np.sign(y + x)

        # Clamp them to the max speed
        self.drive_train.front_left = self.max_speed * percent_left
        self.drive_train.back_left = self.max_speed * percent_left
        self.drive_train.front_right = self.max_speed * percent_right
        self.drive_train.back_right = self.max_speed * percent_right

        # Publish drive_trian
        self.drive_train_publisher.publish(self.drive_train)

    def flipper_position_update(self) -> None:
        """
        Uses saved joystick values to publish new the flipper positions.
        """

        if abs(self.joystick.stick_right_y) > self.deadzone:
            # Move flipper values based on joystick
            self.flipper_position.front_left += float(self.joystick.stick_right_y * self.joystick.bumper_left * self.flipper_sensitivity)
            self.flipper_position.front_right += float(self.joystick.stick_right_y * self.joystick.bumper_right * self.flipper_sensitivity)
            self.flipper_position.back_left += float(self.joystick.stick_right_y * (self.joystick.trigger_left > self.deadzone) * self.flipper_sensitivity)
            self.flipper_position.back_right += float(self.joystick.stick_right_y * (self.joystick.trigger_right > self.deadzone) * self.flipper_sensitivity)

            # Clip them if above or below min max
            self.flipper_position.front_left = float(np.clip(self.flipper_position.front_left, self.flipper_min, self.flipper_max))
            self.flipper_position.front_right = float(np.clip(self.flipper_position.front_right, self.flipper_min, self.flipper_max))
            self.flipper_position.back_left = float(np.clip(self.flipper_position.back_left, self.flipper_min, self.flipper_max))
            self.flipper_position.back_right = float(np.clip(self.flipper_position.back_right, self.flipper_min, self.flipper_max))

        # Send to zero if start is pressed
        if self.joystick.button_start:
            self.flipper_position.front_left += float(-np.sign(self.flipper_position.front_left) * self.flipper_sensitivity)
            self.flipper_position.front_right += float(-np.sign(self.flipper_position.front_right) * self.flipper_sensitivity)
            self.flipper_position.back_left += float(-np.sign(self.flipper_position.back_left) * self.flipper_sensitivity)
            self.flipper_position.back_right += float(-np.sign(self.flipper_position.back_right) * self.flipper_sensitivity)

        # Publish flipper_position
        self.flipper_position_publisher.publish(self.flipper_position)

    #TODO: arm_position_updata(self) - DUSTIN STUFF
    def arm_position_update(self) -> None:
        pass


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
