#!/usr/bin/python3

"""
stream_control.py

Desc: Uses the CameraStream library to host the camera streams.
Author: Isaac Denning
Date: 10/21/23

THIS HAS BEEN DEPRECATED DO TO NOT WORKING!!!
"""

import rclpy
from rclpy.node import Node
from CameraStream.camera_stream import CameraStream


class StreamControl(Node):
    """
    Sets up the can bus and calls the controllers for the corrisponding parts 
        when (drive, flippper, arm) when ROS data is recieved.
    """
    def __init__(self) -> None:
        super().__init__('stream_control')
        self.cameras = CameraStream()
        

    def start(self) -> None:
        """
        Starts the camera streams
        """
        self.cameras.prune_containers()
        self.cameras.startup()

    def stop(self) -> None:
        """
        Stops the camera streams
        """
        self.cameras.shutdown()
        self.cameras.prune_containers()
        


def main(args=None):
    rclpy.init(args=args)

    # Create the node
    stream_control = StreamControl()

    # Start streams
    stream_control.start()

    # Run the node
    try:
        rclpy.spin(stream_control)
    except KeyboardInterrupt:
        pass

    # Destroy it when done
    stream_control.stop()
    stream_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
