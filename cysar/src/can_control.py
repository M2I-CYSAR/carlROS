#!/usr/bin/python3

import rclpy
from rclpy.node import Node
from cysar.msg import FlipperPosition, DriveTrain
from SparkCANLib import SparkController, SparkCAN
from drive_control import DriveControl
from flipper_control import FlipperControl


class CanControl(Node):

    def __init__(self):
        super().__init__('can_control')
        self.bus = SparkCAN.SparkBus(channel="can0", bustype='socketcan', bitrate=1000000)
        self.flipper_control = FlipperControl(self.bus)
        self.drive_control = DriveControl(self.bus)
        self.flipper_subscription = self.create_subscription(FlipperPosition, 'flipper_position', self.flipper_listener, 10)
        self.drive_train_subscription = self.create_subscription(DriveTrain, 'drive_train', self.drive_listener, 10)

    def flipper_listener(self, msg):
        self.flipper_control.set_positions(msg)

    def drive_listener(self, msg):
        self.drive_control.set_speeds(msg)
        


def main(args=None):
    rclpy.init(args=args)

    can_control = CanControl()

    rclpy.spin(can_control)

    can_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()