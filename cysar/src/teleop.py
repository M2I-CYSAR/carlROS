#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from cysar.msg import Joystick, DriveTrain, FlipperPosition, ArmPosition
import numpy as np

deadzone = 0.05
max_speed = 1
flipper_max_pos = -500
flipper_min_pos = 500
flipper_sensitivity = 10

if flipper_max_pos < flipper_min_pos:
    flipper_max_pos, flipper_min_pos = flipper_min_pos, flipper_max_pos

class Teleop(Node):

    def __init__(self):
        super().__init__('Teleop')
        self.joystick = Joystick()
        self.drive_train = DriveTrain()
        self.flipper_position = FlipperPosition()
        self.flipper_position.front_left = 0
        self.flipper_position.front_right = 0
        self.flipper_position.back_left = 0
        self.flipper_position.back_right = 0
        self.drive_train_publisher = self.create_publisher(DriveTrain, 'drive_train', 10)
        self.flipper_position_publisher = self.create_publisher(FlipperPosition, 'flipper_position', 10)
        self.joystick_subscription = self.create_subscription(Joystick, 'joystick', self.listener, 10)

    def listener(self, msg):
        self.joystick = msg
        self.talker()

    def talker(self):
        self.drive_train_update()
        self.flipper_position_update()

    def drive_train_update(self):
        # Unnecessarily complex equation for finding velocity of motors based on joystick
        x = self.joystick.stick_left_x if abs(self.joystick.stick_left_x) > deadzone else 0
        y = self.joystick.stick_left_y if abs(self.joystick.stick_left_y) > deadzone else 0
        r2o2 = np.sqrt(2) / 2
        dist = np.sqrt(x*x + y*y)
        normX = x / dist if dist != 0 else 0
        normY = y / dist if dist != 0 else 0
        percent_left = np.minimum(1, np.arccos(np.minimum(abs(normX * r2o2 + normY * r2o2), 1)) / np.pi * 2 * dist) * np.sign(y - x)
        percent_right = np.minimum(1, np.arccos(np.minimum(1, abs(normX * -r2o2 + normY * r2o2))) / np.pi * 2 * dist) * np.sign(y + x)

        # Clamp them to the max speed
        self.drive_train.front_left = max_speed * percent_left
        self.drive_train.back_left = max_speed * percent_left
        self.drive_train.front_right = max_speed * percent_right
        self.drive_train.back_right = max_speed * percent_right

        # Publish drive_trian
        self.drive_train_publisher.publish(self.drive_train)

    def flipper_position_update(self):
        # Move flipper values based on joystick
        self.flipper_position.front_left += int(self.joystick.stick_right_y * self.joystick.bumper_left * flipper_sensitivity)
        self.flipper_position.front_right += int(self.joystick.stick_right_y * self.joystick.bumper_right * flipper_sensitivity)
        self.flipper_position.back_left += int(self.joystick.stick_right_y * (self.joystick.trigger_left > deadzone) * flipper_sensitivity)
        self.flipper_position.back_right += int(self.joystick.stick_right_y * (self.joystick.trigger_right > deadzone) * flipper_sensitivity)

        # Clip them if above or below min max
        self.flipper_position.front_left = int(np.clip(self.flipper_position.front_left, flipper_min_pos, flipper_max_pos))
        self.flipper_position.front_right = int(np.clip(self.flipper_position.front_right, flipper_min_pos, flipper_max_pos))
        self.flipper_position.back_left = int(np.clip(self.flipper_position.back_left, flipper_min_pos, flipper_max_pos))
        self.flipper_position.back_right = int(np.clip(self.flipper_position.back_right, flipper_min_pos, flipper_max_pos))

        # Send to zero if start is pressed
        if self.joystick.button_start:
            self.flipper_position.front_left = 0
            self.flipper_position.front_right = 0
            self.flipper_position.back_left = 0
            self.flipper_position.back_right = 0

        # Publish flipper_position
        self.flipper_position_publisher.publish(self.flipper_position)

def print_msg(msg):
    print(f'({type(msg)})')

    if not hasattr(msg, "get_fields_and_field_types"):
        return

    fields = msg.get_fields_and_field_types()
    # Iterate through the dictionary and print the values
    for field, field_type in fields.items():
        if hasattr(msg, field):
            print(f'{field}: {getattr(msg, field)}')

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