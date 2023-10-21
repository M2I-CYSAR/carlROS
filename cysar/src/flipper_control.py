#!/usr/bin/python3

# CAN IDs for Flipper Controllers
FLF = 21
FRF = 22
BLF = 23
BRF = 24

class Flipper:
    # Create a new controller for Flipper Motor Controller
    def __init__(self, bus, id):
        self.controller = bus.init_controller(id)
        self.home = 0
        self.set_home()

    def set_home(self):
        self.home = self.controller.position

    def go_home(self):
        self.controller.position_output(self.home)

    def get_position(self):
        return self.controller.position

    def rotate_flipper_position(self, position):
        self.controller.position_output(position + self.home)


class FlipperControl():
    def __init__(self, bus):
        self.FLFlipper = Flipper(bus, FLF)
        self.FRFlipper = Flipper(bus, FRF)
        self.BLFlipper = Flipper(bus, BLF)
        self.BRFlipper = Flipper(bus, BRF)

    def set_positions(self, msg):
        self.FLFlipper.rotate_flipper_position(msg.front_left)
        self.FRFlipper.rotate_flipper_position(msg.front_right)
        self.BLFlipper.rotate_flipper_position(msg.back_left)
        self.BRFlipper.rotate_flipper_position(msg.back_right)
