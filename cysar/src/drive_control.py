#!/usr/bin/python3

# CAN IDs for Drive Controllers
FLD = 11
FRD = 12
BLD = 13
BRD = 14

class DriveControl():
    # Create a new controller for Drive Motor Controller
    def __init__(self, bus):
        self.bus = bus
        self.FLMotor = self.bus.init_controller(FLD)
        self.FRMotor = self.bus.init_controller(FRD)
        self.BLMotor = self.bus.init_controller(BLD)
        self.BRMotor = self.bus.init_controller(BRD)

    def set_speeds(self, msg):
        self.FLMotor.percent_output(msg.front_left)
        self.FRMotor.percent_output(msg.front_right)
        self.BLMotor.percent_output(msg.back_left)
        self.BRMotor.percent_output(msg.back_right)
