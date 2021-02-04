#!/usr/bin/env python

from __future__ import print_function
import glob
from rosSorter.srv import gCodeCommand
import rospy
import serial
import time

#connect to SKR over serial interface
SKR = glob.glob('/dev/ttyACM*')[0]
s = serial.Serial(SKR,115200)

# Wake up SKR Board
s.write("\r\n\r\n") # Hit enter a few times to wake the Printrbot
time.sleep(3)   # Wait for Printrbot to initialize
s.flushInput()  # Flush startup text in serial input

def sendGCode(req):
    rospy.logdebug(req)
    print("req.gCodeCommand")
    s.write(req.gCodeCommand + '\n') # send gCode to SKR
    rospy.logdebug(s.readline()) # wait for response
    # TODO error handling
    return []

def gcodeInterface_server():
    rospy.init_node('gcodeSender_node')
    s = rospy.Service('gCodeSender', gCodeCommand, sendGCode)
    rospy.spin()

rospy.on_shutdown(s.close);

if __name__ == "__main__":
    gcodeInterface_server()
