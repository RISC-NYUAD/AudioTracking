#!/usr/bin/env python
import os,sys
import rospy
import numpy as np
import time

from std_msgs.msg import Float32MultiArray

from utils import *

if os.name == 'nt':
    import msvcrt
    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

import signal
from dynamixel_sdk import * # Uses Dynamixel SDK library

MY_DXL = 'X_SERIES'
ADDR_TORQUE_ENABLE          = 64
ADDR_GOAL_POSITION          = 116
ADDR_PRESENT_POSITION       = 132
ADDR_PRO_PRESENT_VELOCITY   = 128
DXL_MINIMUM_POSITION_VALUE  = 1500         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE  = 2000         # Refer to the Maximum Position Limit of product eManual
BAUDRATE                    = 1000000

ADDR_TORQUE_ENABLE2          = 64
ADDR_GOAL_POSITION2          = 116
ADDR_PRESENT_POSITION2       = 132
DXL_MINIMUM_POSITION_VALUE2  = 1700         # Refer to the Minimum Position Limit of product eManual
DXL_MAXIMUM_POSITION_VALUE2  = 2300      # Refer to the Maximum Position Limit of product eManual
BAUDRATE                     = 1000000

# DYNAMIXEL Protocol Version (1.0 / 2.0)
# https://emanual.robotis.com/docs/en/dxl/protocol2/
PROTOCOL_VERSION            = 2.0

# Factory default ID of all DYNAMIXEL is 1
DXL_ID                      = 11
DXL_ID2                     = 12

# Use the actual port assigned to the U2D2.
# ex) Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"
DEVICENAME                  = '/dev/ttyUSB0'

TORQUE_ENABLE               = 1     # Value for enabling the torque
TORQUE_DISABLE              = 0     # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold

# sr = 96000
# channels = 16
# pan_tilt_angles = Float32MultiArray()
# bridge = CvBridge()

def handler(signum, frame):
    res = input("Ctrl-c was pressed. Do you really want to exit? y/n ")
    if res == 'y':
        # Disable Dynamixel Torque
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID2, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        # Close port
        portHandler.closePort()
        exit(1)

def camera_pan_tilt(angles):
    
    in_data = np.array(angles.data)
    lat, lon = in_data
    print(in_data.shape)
    pan_angle = int((lon+20) / 180 * 2048 + 1024)
    til_angle = int(2048 - lat / 120 * 1024)


    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, pan_angle)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, ADDR_GOAL_POSITION2, til_angle)
    time.sleep(0.1)

    rospy.loginfo("pan and tilt camera")

if __name__ == "__main__":
    rospy.init_node('pan_tilt', anonymous=True)

    # Initialize PortHandler instance
    # Set the port path
    # Get methods and members of PortHandlerLinux or PortHandlerWindows
    portHandler = PortHandler(DEVICENAME)

    # Initialize PacketHandler instance
    # Set the protocol version
    # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        print("Press any key to terminate...")
        getch()
        quit()


    # Set port baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        print("Press any key to terminate...")
        getch()
        quit()

    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel has been successfully connected")

    # Enable Dynamixel Torque
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, DXL_ID2, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
    if dxl_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
    elif dxl_error != 0:
        print("%s" % packetHandler.getRxPacketError(dxl_error))
    else:
        print("Dynamixel has been successfully connected on device 2")

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, 112, 1000)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,  108, 1000)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, 108, 1000)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,  112, 1000)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, 84, 200)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,  84, 200)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, 48, 1600)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,  48, 1600)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, 52, 2300)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,  52, 2300)

    signal.signal(signal.SIGINT, handler)

    dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION)
    print("current position pan", dxl_present_position)
    dxl2_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(portHandler, DXL_ID2, ADDR_PRESENT_POSITION)
    print("current position tilt", dxl2_present_position)

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, 84, 800)
    #dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, 82, 50)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID,  84, 800)

    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, 2048)
    dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(portHandler, DXL_ID2, ADDR_GOAL_POSITION2, 2048)
    time.sleep(1)

    rospy.Subscriber("/audio/pantiltangles", Float32MultiArray, camera_pan_tilt)
    rospy.spin()    
    