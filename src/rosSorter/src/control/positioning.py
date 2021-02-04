#!/usr/bin/env python

from __future__ import print_function
import glob
from rosSorter.srv import positioning
from rosSorter.srv import gCodeCommand
import rospy
import time

# Constants
SIZE_X = 800
SIZE_Y = 730

DIST_X = 35
DIST_Y = 68

BOXES_X = int(SIZE_X / DIST_X)
BOXES_Y = int(SIZE_Y / DIST_Y)

# setup ROS service handles
sendGCode = rospy.ServiceProxy('gCodeSender', gCodeCommand)

def handle_positioning(req):
    rospy.logdebug(req)
    if req.mode == 0:
        sendGCode("G0 X" + str(req.x * DIST_X) + " Y" + str(req.y * DIST_Y)) # move to grid position
    else:
        sendGCode("G0 X" + str(req.x) + " Y" + str(req.y)) # move to coordinates
    return []

def positioning_server():
    print("waiting for gCodeSender")
    rospy.wait_for_service('gCodeSender')
    print("service staring")
    rospy.init_node('positioning_node')
    xaccel = rospy.get_param('~X_acceleration')
    yaccel = rospy.get_param('~Y_acceleration')
    print(yaccel)
    print(xaccel)
    sendGCode("G0 F 99999" + '\n') # initialize feedrate for G0
    sendGCode("M201 X" + str(xaccel) + " Y" + str(yaccel)) # initialize acceleration
    sendGCode("M42 P10 M1" + '\n') # set pin 10 on SKR to output
    sendGCode("M42 P10 S0" + '\n') # set pin 10 initially to LOW
    sendGCode("G28 X Y" + '\n') # home grabber
    sendGCode("G0 X12 Y30" + '\n') # move grabber out of the collision zone
    s = rospy.Service('position_sled', positioning, handle_positioning)
    rospy.spin()

if __name__ == "__main__":
    positioning_server()
