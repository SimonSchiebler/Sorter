#!/usr/bin/env python
# this is the main node of the robot.
# calling this node will trigger the whole memory card detection and solving process.
# This node also does the matching controls the grabbing of the cards. 

from __future__ import print_function
from rosSorter.srv import scan
from rosSorter.srv import extractCards
from rosSorter.srv import positioning
from rosSorter.srv import grabber
from std_srvs.srv import Trigger
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge
bridge = CvBridge()

# Constants
scanHandle = rospy.ServiceProxy('scan_Field', scan)
extractCardsHandle = rospy.ServiceProxy('extract', extractCards)
positioningHandle = rospy.ServiceProxy('position_sled', positioning)
grabberHandle = rospy.ServiceProxy('grab', grabber)

GRABBER_X_OFFSET = 50
GRABBER_Y_OFFSET = -25

SIZE_X = 800
SIZE_Y = 730

DIST_X = 35
DIST_Y = 68

BOXES_X = int(SIZE_X / DIST_X)
BOXES_Y = int(SIZE_Y / DIST_Y)

SCAN_FROM_X = 3
SCAN_FROM_Y = 2
SCAN_TO_X = 18
SCAN_TO_Y = 8

# detects pairs of cards in an image using the SIFT algorithm
def matchCards(img):
    sift = cv2.xfeatures2d.SIFT_create()
    flann = cv2.FlannBasedMatcher({"algorithm":0,"trees":0},{})
    extractedCards = extractCardsHandle(bridge.cv2_to_compressed_imgmsg(img))
    extractedCards = extractedCards.cards
    
    # creates a easier to use data object for the cards and calculates the SIFT descriptors
    extractedCards = map(lambda x: {'xCoord':x.xCoord, 'yCoord':x.yCoord, 'image': bridge.imgmsg_to_cv2(x.image)}, extractedCards)
    extractedCards = map(lambda x: {'xCoord':x['xCoord'], 'yCoord':x['yCoord'],'image': x['image'], 'keyPoints':sift.detectAndCompute(x['image'], None)   }, extractedCards)
    extractedCards = map(lambda x: {'xCoord':x['xCoord'], 'yCoord':x['yCoord'],'image': x['image'], 'keyPoints':x['keyPoints'][0],'descriptor':x['keyPoints'][1]}, extractedCards)
    
    pairs = []
    while len(extractedCards) > 0:
        similarities = []
        for index, cardB in enumerate(extractedCards[1:]):
            # calculates a matching score for each possible card pair
            matches = flann.knnMatch(extractedCards[0]["descriptor"],cardB["descriptor"],k=2) 
            total = 0
            for match in matches:
                total = total + match[0].distance
            diff = total / len(matches)
            similarities.append({'diff': diff, 'card': cardB})
        
        similarities.sort(key=lambda x: x['diff']) 
        if len(similarities) > 0 and similarities[0]['diff'] < 600: # thresholding to exclude false positive matches 
            pairs.append([extractedCards[0], similarities[0]['card']])
            del extractedCards[extractedCards.index(similarities[0]['card'])]
        del extractedCards[0]
    return pairs

def pixelToGrabberCoord(x,y,img): # translates image coordinates to robot positions
    height, width, dim = img.shape
    heightTranslationFactor =  float(SIZE_Y - (SCAN_FROM_Y + BOXES_Y - SCAN_TO_Y)*DIST_Y) / height 
    widthTranslationFactor = float(SIZE_X - (SCAN_FROM_X + BOXES_X - SCAN_TO_X)*DIST_X) / width
    xPixelsFromOrigin = width - x
    yPixelsFromOrigin = height - y
    return xPixelsFromOrigin*widthTranslationFactor+GRABBER_X_OFFSET+SCAN_FROM_X+DIST_X, yPixelsFromOrigin*heightTranslationFactor+GRABBER_Y_OFFSET+SCAN_FROM_Y+DIST_Y

# iterates over pairs and controls the sled to stack them one over another.
def stackPairs(pairs, img): 
    for pair in pairs:
        cv2.imwrite("currentPair.jpg", np.concatenate([pair[0]["image"],pair[1]["image"]],axis=1)) # done for logging purposes
        xGrabCoord, yGrabCoord = pixelToGrabberCoord(pair[0]['xCoord'],pair[0]['yCoord'],img)
        positioningHandle(xGrabCoord, yGrabCoord,1)
        grabberHandle(0, 0, 1000)
        xGrabCoord, yGrabCoord = pixelToGrabberCoord(pair[1]['xCoord'],pair[1]['yCoord'],img)
        positioningHandle(xGrabCoord, yGrabCoord ,1)
        grabberHandle(1, 0, 1000)

def solve(req):
    img = bridge.compressed_imgmsg_to_cv2(scanHandle(1,1,18,8).img)
    cv2.imwrite("out.jpg", img)
    pairs = matchCards(img)
    stackPairs(pairs, img)
    return []


def solver_server():
    print("waiting for scan_Field")
    rospy.wait_for_service('scan_Field')
    print("waiting for extract")
    rospy.wait_for_service('extract')
    print("waiting for grabber")
    rospy.wait_for_service('grab')
    print("waiting for positioning")
    rospy.wait_for_service('position_sled')
    
    print("service staring")
    rospy.init_node('memorySolver')
    s = rospy.Service('solveMemory', Trigger, solve)
    rospy.spin()

if __name__ == "__main__":
    solver_server()
