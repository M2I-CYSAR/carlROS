#!/usr/bin/python3

from SparkCAN import SparkBus

# CAN IDs for Drive Controllers
FLD = 11
FRD = 21
RLD = 31
RRD = 41

# Factor to scale Drive Speed
SCALE = 0.75

#Instantiate SparkBus object
bus = SparkBus(channel="can0", bustype='socketcan', bitrate=1000000)

# Create a new controllers for Drive Motor Controllers
# Front Left Drive SparkMax
sparkFLD = bus.init_controller(FLD)
# Front Right Drive SparkMax
sparkFRD = bus.init_controller(FRD)
# Rear Left Drive SparkMax
sparkRLD = bus.init_controller(RLD)
# Rear Right Drive SparkMax
sparkRRD = bus.init_controller(RRD)

def arcade_drive(rotate, drive):
    """Drives the robot using arcade drive."""
    # variables to determine the quadrants
    maximum = max(abs(drive), abs(rotate)) * SCALE
    total = (drive + rotate) * SCALE
    difference = (drive - rotate) * SCALE

    # set speed according to the quadrant that the values are in
    if drive >= 0:
        if rotate >= 0:  # I quadrant
            sparkFLR.percent_output(maximum)
            sparkRLD.percent_output(maximum)

            sparkFRR.percent_output(difference)
            sparkRRD.percent_output(difference)

        else:            # II quadrant
            sparkFLD.percent_output(total)
            sparkRLD.percent_output(total)

            sparkFRR.percent_output(maximum)
            sparkRRD.percent_output(maximum)

    else:
        if rotate >= 0:  # IV quadrant
            sparkFLD.percent_output(total)
            sparkRLD.percent_output(total)

            sparkFRR.percent_output(-maximum)
            sparkRRD.percent_output(-maximum)

        else:            # III quadrant
            sparkFLD.percent_output(-maximum)
            sparkRLD.percent_output(-maximum)

            sparkFRR.percent_output(difference)
            sparkRRD.percent_output(difference)
