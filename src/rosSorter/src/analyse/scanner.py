#!/usr/bin/env python

# this node creates a scan of the whole working area by positioning 
# the sled and taking images.
# The images are then stitched together and returned as one big image.

from __future__ import print_function
import glob
from rosSorter.srv import scan
from rosSorter.srv import gCodeCommand
from rosSorter.srv import positioning
import numpy as np
import RPi.GPIO as GPIO
import rospy
import time
from helper import unwarpImage, cropVisiblePart
import cv2
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge
bridge = CvBridge()

# Constants
# TODO Centralize these Constants
BOXES_X = int(800 / 41)
BOXES_Y = int(730 / 75)
CAMERA_IMAGE_TOPIC = "/raspicam_node/image/compressed"
CORRECTION_MATRIX = np.load("corr.npy") # corr.npy is calculated by ./illuminationCorrection.py 
PI_CALLBACK_PIN = 18
SKR_CALLBACK_PIN = 326

# setup ROS service handles
sendGCode = rospy.ServiceProxy('gCodeSender', gCodeCommand)
position = rospy.ServiceProxy('position_sled', positioning)

def performScan(req):
    images = []
    assert ((req.xDiff + req.xRange) <= BOXES_X)
    assert ((req.yDiff + req.yRange) <= BOXES_Y)
    rospy.logdebug(req)
    for xpos in range(0,req.xRange):
        images.append([])
        for ypos in range(0,req.yRange):
            position(xpos + req.xDiff,ypos + req.yDiff, 0)
            sendGCode("M400") # wait for all moves to finish
            sendGCode("M42 P" + SKR_CALLBACK_PIN + " S255" + '\n') # set SKR_CALLBACK_PIN to HIGH
            while not GPIO.input(PI_CALLBACK_PIN):
                time.sleep(0.1)
            sendGCode("M42 P" + SKR_CALLBACK_PIN + " S0" + '\n') # set SKR_CALLBACK_PIN to LOW
            img = rospy.wait_for_message(CAMERA_IMAGE_TOPIC, CompressedImage, 5000)
            img = bridge.compressed_imgmsg_to_cv2(img, 'bgr8')
            img = cropVisiblePart(img) # crops the image to exclude reflecting parts.
            img = unwarpImage(img) # applies fisheye correction
            img = (img + CORRECTION_MATRIX).clip(max=255)
            img = cv2.resize(img, (0,0),fx=0.5, fx=0.5)
            images[xpos].append(img);
    imagerows = []
    rownum = 0;
    for row in images:
        row.reverse()
        rowimg = np.concatenate(row, axis=0)
        rownum += 1;
        imagerows.append(rowimg)
    imagerows.reverse()
    finishedImg = np.concatenate(imagerows, axis=1)
    return bridge.cv2_to_compressed_imgmsg(finishedImg)

def scanner_server():
    print("waiting for positioning")
    rospy.wait_for_service('position_sled')
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(PI_CALLBACK_PIN, GPIO.IN,pull_up_down=GPIO.PUD_DOWN)
    sendGCode("M42 P" + SKR_CALLBACK_PIN + " M1" + '\n') # set SKR_CALLBACK_PIN to output mode
    sendGCode("M42 P" + SKR_CALLBACK_PIN + " S0" + '\n') # set SKR_CALLBACK_PIN to LOW
    print("service staring")
    rospy.init_node('scanner')
    s = rospy.Service('scan_Field', scan, performScan)
    rospy.spin()

if __name__ == "__main__":
    scanner_server()
