#!/usr/bin/python3

"""
teleop.py

Desc: Main control loop for carl operation
Author: Cole Hunt
Date: 4/14/23

"""

from driveTrain import DriveTrain
from SparkCANLib import SparkCAN
from flipperControl import FlipperControl
from GameControllerLib import Gamepad
import time

ENABLED = True

def main():

    # Gamepad settings
    gamepadType = Gamepad.Xbox360
    joystickSpeed = 'LEFT-Y'
    joystickSteering = 'LEFT-X'

    flipperHome = 'B'
    flipperRaise ='Y'
    flipperLower = 'A'

    leftBumper = 'LB'
    rightBumper = 'RB'
    leftTrigger = 'LT'
    rightTrigger = 'RT'

    # Wait for a connection
    if not Gamepad.available():
        print('Please connect your gamepad...')
        while not Gamepad.available():
            time.sleep(1.0)
    gamepad = gamepadType()
    print('Gamepad connected')

    #Instantiate SparkBus object
    bus = SparkCAN.SparkBus(channel="can0", bustype='socketcan', bitrate=1000000)

    driveTrainObj = DriveTrain(bus)
    #flipperControlObj = FlipperControl(bus)

    steering = 0
    speed = 0

    while(ENABLED):
        eventType, control, value = gamepad.getNextEvent()
        if control == joystickSpeed:
            # Speed control (inverted)
            speed = -value
        elif control == joystickSteering:
            # Steering control (not inverted)
            steering = value
        #print(f"Speed: {speed}, Steer: {steering}")
        driveTrainObj.arcadeDrive(speed, steering)

if __name__ == "__main__":
    main()
