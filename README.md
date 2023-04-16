# CySAR - CARL
## Description
Certified. Autonomous. (with) Robotic. Legs.
CARL is the current robot in development by the MSI CysSAR team. This repo is for the onboard software for the robot to be run on the Jetson.

The robot consists of 4 flippers with treads to allow multi terrain naviagtion. The robot operations in an arcade drive manner. This has driving mapped to one joystick with the Y-Axis as Speed and the X-Axis as Steering. The flippers are controlled independently to allow for maximum flexibility and are position based.

## CANable Setup
The robot runs of Rev Spark Max motorcontrollers all of which are controller of the CAN bus. The Jetson communicates over CAN using the CANable USB Adapter. The repo includes a script to setup the CANable *can0setup.sh*. This script **must** be run in a Linux enviroment and will **not** work in a Windows enviroment.

#### can0 Interface Setup
1. Open a terminal and navigate to the project directory
    - [Shell Navigation Commands](https://www.guru99.com/linux-commands-cheat-sheet.html)
2. Plug in CANable into a USB port on the driving device
3. Run the script as superuser using `sudo ./can0setup` in the terminal
4. You can confirm the `can0` interface is setup by running `ifconfig` in the terminal. You will see a list of configured interface on your machine one of which should be named `can0`

#### Setup Testing
The adapter should be setup on the system and can be tested by doing the following
1. Open a new termianl and run `candump can0`
    - This should show all traffic received by can0. The terminal will be blocked while the candump is running while waiting for messages
2. Then in a different terminal run `cansend can0 999#DEADBEEF`
    - This will send a frame to `0x999` with payload `0xdeadbeef` and you should see the message appear in the candump terminal
3. After this test ensure to close the candump terminal using `Ctrl-C` to ensure the machine is not reading all CAN messages to the robot.

## Teleop Control
The primary or *main* file of the program is *teleop.py*. This file is setup to be an executable that will run the primary controll loop for the robot.

#### Enabling the Robot
To start the primary control loop the following steps need to be complete.

**WARNING: These steps do not include any of the required actions to power on the robot. Please consult the System Team Checklist for Robot Opertaion and Configuration**

1. The current program only takes and Xbox 360 controller as input. Please ensure you have connected an Xbox 360 controller to the device through USB.
    - *TODO: Add Multi-Controller Support (Xbox One, DS4, DS3, etc)*
2. Confirm that the CANable is connected to the driver device and that the CAN bus end has been attached to the robot CAN bus.
3. The Robot **must** be turned **on** to proceed with further instructions.
    - If the Robot in not powered on the CANable will attempt to to read and recieve data and cause a buffer overflow.

**WARNING: Once the next step is run the robot will be live and will attempt to react to Joystick inputs. Do not continue if this not desired**

4. The robot can now be enabled by running `./teleop.py`
    - The program will wait for a controller to be detect and then begin to send CAN messages to the robot executing the desired actions
