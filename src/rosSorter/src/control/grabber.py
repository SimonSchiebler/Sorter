#!/usr/bin/env python

from __future__ import print_function
import glob
from rosSorter.srv import grabber
from rosSorter.srv import gCodeCommand
import rospy

# Constants
DEFAULT_ACTUATION_TIME = 500
DEFAULT_AIRREGULATION_TIME = 3000
SOLENOID_PIN = 325
VACUUM_PIN = 131

# setup ROS service handles
sendGCode = rospy.ServiceProxy('gCodeSender', gCodeCommand)

def setPin(pinNumber, state):
    sendGCode("M42 P" + str(pinNumber) + " S" + str(state) + '\n')

def sleepOnSKR(ms):
    sendGCode("G4 P" + str(ms) + '\n')


def handle_grab(data):
    rospy.logdebug(data)
    
    # set default variables
    data.actuationtime = data.actuationtime if data.actuationtime else DEFAULT_ACTUATION_TIME 
    data.airregulationtime = data.airregulationtime if data.airregulationtime else DEFAULT_AIRREGULATION_TIME

    sendGCode("M400") # wait for all moves to finish

    # Grab card
    if data.command == 0:
        setPin(VACUUM_PIN, 244)
        setPin(SOLENOID_PIN,244)
        
    # place card
    if data.command == 1:
        setPin(SOLENOID_PIN,244)
        setPin(VACUUM_PIN,0)
	
    sleepOnSKR(data.airregulationtime + data.actuationtime)
    setPin(SOLENOID_PIN,0)
    sleepOnSKR(data.actuationtime)
    
    return []

def grabber_server():
    print("waiting for gCodeSender")
    rospy.wait_for_service('gCodeSender')
    print("service staring")
    rospy.init_node('grabber_node')
    s = rospy.Service('grab', grabber, handle_grab)
    rospy.spin()

if __name__ == "__main__":
    grabber_server()
