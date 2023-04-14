"""
flipperControl.py

Desc: Control Methods for flipper control
Author: Cole Hunt
Date: 4/14/23

"""

from SparkCANLib import SparkController

# CAN IDs for Flipper Controllers
FLF = 21
FRF = 22
RLF = 23
RRF = 24

# Factor to scale Rotate Speed
SCALE = 0.25

class Flipper:
    # Create a new controller for Flipper Motor Controller
    def __init__(self, bus, id):
        self.controller = self.bus.init_controller(id)
        self.setHome()

    def setHome(self):
        self.home = self.position

    def goHome(self):
        self.controller.position_output(self.home)

    def setThresholds(self, upper, lower):
        self.upper = upper
        self.lower = lower

    def inThreshold(self):
        if((self.controller.position > self.upper) or (self.controller.position < self.lower)):
            return False
        else:
            return True

    def raiseFlipper(self, control):
        if(self.inThreshold()):
            self.controller.position_output(self.controller.postion + control)

    def lowerFlipper(self, control):
        if(self.inThreshold()):
            self.controller.position_output(self.controller.postion - control)

class FlipperControl:

    def __init__(self, bus):
        self.bus = bus
        # Front Left Drive Flipper
        self.FLFlipper = Flipper(bus, FLF)
        # Front Right Drive Flipper
        self.FRFlipper = Flipper(bus, FRF)
        # Rear Left Drive Flipper
        self.RLFlipper = Flipper(bus, RLF)
        # Rear Right Drive Flipper
        self.RRFlipper = Flipper(bus, RRF)

    def returnToHome(self):
        self.FLFlipper.goHome()
        self.FRFlipper.goHome()
        self.RLFlipper.goHome()
        self.RRFlipper.goHome()

    def raiseSystem(self, fl, fr, rl, rr):
        self.FLFlipper.raiseFlipper(fl)
        self.FRFlipper.raiseFlipper(fr)
        self.RLFlipper.raiseFlipper(rl)
        self.RRFlipper.raiseFlipper(rr)

    def lowerSystem(self, fl, fr, rl, rr):
        self.FLFlipper.lowerFlipper(fl)
        self.FRFlipper.lowerFlipper(fr)
        self.RLFlipper.lowerFlipper(rl)
        self.RRFlipper.lowerFlipper(rr)


