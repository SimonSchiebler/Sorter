#!/usr/bin/env python
# This module is not part of the robot logic.
# It is used to calculate a matrix that corrects the illumination dispersion 
# created by the light source.

from sensor_msgs.msg import Image
import cv2
import numpy as np

# black.jpg is a picture of a uniformly black surface taken by the robots camera.
# it must have the size of a part image from a scan after being corrected and cropped.
img = cv2.imread("black.jpg") 

B = img[:][:][0]
G = img[:][:][1]
R = img[:][:][2]

maxarr = np.full(img.shape, [B.max(),G.max(),R.max()])
minarr = np.full(img.shape, [B.min(),G.min(),R.min()])

corrarr = (maxarr - img) +minarr

np.save("corr.npy", corrarr)
cv2.imwrite("corr.jpg", img + corrarr)