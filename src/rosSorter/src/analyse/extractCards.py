#!/usr/bin/env python

# this node takes a stitched picture of the whole 
# workarea and extracts all cards in it.
# it then returns an array of all cards with their respectice 
# coordinates in the picture

from __future__ import print_function
from rosSorter.srv import extractCards
from rosSorter.srv import extractCardsResponse
from rosSorter.msg import Card
import numpy as np
import rospy
from cv_bridge import CvBridge
import cv2
from sensor_msgs.msg import CompressedImage
bridge = CvBridge()

from rosSorter.srv import scan

# setup ROS service handles
scan = rospy.ServiceProxy('scan_Field', scan)

def extract(req):
    imgOrig = bridge.compressed_imgmsg_to_cv2(req.img)

    imgArr = []
    img = imgOrig.copy()

    # image preparation
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    imgArr.append(img)
    imgGauss = cv2.GaussianBlur(img, (5,5), 0)
    imgArr.append(imgGauss)
    threshold1 = 80
    asd, thresh = cv2.threshold(imgGauss,threshold1,255,cv2.THRESH_BINARY)
    imgArr.append(thresh)
    cv2.imwrite("thresh.jpg", thresh)
    imgMed = cv2.medianBlur(thresh,5)
    imgArr.append(imgMed)
    cv2.imwrite("medBlur.jpg", imgMed)

    # card contour recognition
    minsize = 100000 # these values refer to the size of a memory card. They refer to the number of pixels in a card.
    maxsize = 1441680.0 /4 
    baseImg = np.zeros(imgMed.shape)
    image, contours, hierarchy = cv2.findContours(imgMed, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # loops over all contours and filters them based on size to extract all cards.
    cards = []
    for cnt in contours:
        # correct inaccuracies from contour finding algorithm.
        cnt = cv2.convexHull(cnt)
        cnt = cv2.minAreaRect(cnt)
        cnt = cv2.boxPoints(cnt)

        area = cv2.contourArea(cnt)
        if area >= minsize and area <= maxsize : # filter by size
            approx = np.int0(cnt)
            target = np.float32([[250,250],[250,0],[0,0],[0,250]])
            src = np.float32(approx)
            transform = cv2.getPerspectiveTransform(src,target)
            cardImg = cv2.warpPerspective(imgOrig,transform, (250,250))
            center = sum(approx)/4
            
            histImage = bridge.cv2_to_imgmsg(cardImg);
            card = Card(int(center[0]), int(center[1]), histImage)
            cards.append(card)
    return extractCardsResponse(cards)
    

def scanner_server():
    print("service staring")
    rospy.init_node('extractor')
    s = rospy.Service('extract', extractCards, extract)
    rospy.spin()

if __name__ == "__main__":
    scanner_server()
