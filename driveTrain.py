"""
driveTrain.py

Desc: Method to control drive motors for carl
Author: Cole Hunt
Date: 4/13/23

"""

from SparkCANLib import SparkController

# CAN IDs for Drive Controllers
FLD = 11
FRD = 12
RLD = 13
RRD = 14

# Factor to scale Drive Speed
SCALE = 1.00

# Factor to invert Left Treads
INVERTED = -1
class DriveTrain:

    def __init__(self, bus):
        self.bus = bus
        # Create a new controllers for Drive Motor Controllers
        # Front Left Drive SparkMax
        self.sparkFLD = self.bus.init_controller(FLD)
        # Front Right Drive SparkMax
        self.sparkFRD = self.bus.init_controller(FRD)
        # Rear Left Drive SparkMax
        self.sparkRLD = self.bus.init_controller(RLD)
        # Rear Right Drive SparkMax
        self.sparkRRD = self.bus.init_controller(RRD)

    def arcadeDrive(self, rotate, drive):
        # variables to determine the quadrants
        rotate = -rotate # Invert X Axis for drive control
        maximum = max(abs(drive), abs(rotate)) * SCALE
        total = (drive + rotate) * SCALE
        difference = (drive - rotate) * SCALE

        # set speed according to the quadrant that the values are in
        if drive >= 0:
            if rotate >= 0:  # I quadrant
                self.sparkFLD.percent_output(maximum * INVERTED)
                self.sparkRLD.percent_output(maximum  * INVERTED)

                self.sparkFRD.percent_output(difference)
                self.sparkRRD.percent_output(difference)

            else:            # II quadrant
                self.sparkFLD.percent_output(total * INVERTED)
                self.sparkRLD.percent_output(total * INVERTED)

                self.sparkFRD.percent_output(maximum)
                self.sparkRRD.percent_output(maximum)

        else:
            if rotate >= 0:  # IV quadrant
                self.sparkFLD.percent_output(total * INVERTED)
                self.sparkRLD.percent_output(total * INVERTED)

                self.sparkFRD.percent_output(-maximum)
                self.sparkRRD.percent_output(-maximum)

            else:            # III quadrant
                self.sparkFLD.percent_output(-maximum * INVERTED)
                self.sparkRLD.percent_output(-maximum * INVERTED)

                self.sparkFRD.percent_output(difference)
                self.sparkRRD.percent_output(difference)

    def disable(self):
        self.sparkFLD.percent_output(0)
        self.sparkRLD.percent_output(0)
        self.sparkFRD.percent_output(0)
        self.sparkRRD.percent_output(0)