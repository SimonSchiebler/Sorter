#!/usr/bin/env python

from sensor_msgs.msg import Image
from helper import cropVisiblePart
import rospy
from cv_bridge import CvBridge
import cv2
import numpy as np
import glob
bridge = CvBridge()

DIM=(729, 1232)
K=np.array([[65949.79058215416, 0.0, -743.2075275257644], [0.0, 65780.92244287752, 685.2983318216144], [0.0, 0.0, 1.0]])
D=np.array([[7.21225896261792], [-156752.02943681442], [186651207.96973005], [-70768024746.31644]])

# Constants
CAMERA_IMAGE_TOPIC = "/raspicam_node/image"

def rotate_image(image, angle):
  image_center = tuple(np.array(image.shape[1::-1]) / 2)
  rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
  result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
  return result

imgPaths = glob.glob("scanImg/*.jpg")

(height, width) = cv2.imread(imgPaths[0]).shape[:2]
cv2.namedWindow("Parameters")
cv2.resizeWindow("Parameters", 500, 500)
cv2.createTrackbar("Rotation","Parameters", 0 , 200, lambda x: x)
cv2.createTrackbar("rotPadX","Parameters", 0 , int(width/10), lambda x: x)
cv2.createTrackbar("rotPadY","Parameters", 0 , int(height/10), lambda x: x)

#map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)


images = []
for x in range(0,6):
    images.append([])
    for y in range(0,5):
        images[x].append([])

for path in imgPaths:
    x = path.split(".")[0].split("-")[0].split("/")[-1]
    y = path.split(".")[0].split("-")[1]
    images[int(x)][int(y)] = cv2.imread(path)

for y, imgrow in enumerate(images):
    for x, img in enumerate(imgrow):
        (height, width) = img.shape[:2]
        #pady = cv2.getTrackbarPos("rotPadY", "Parameters")
        #padx = cv2.getTrackbarPos("rotPadX", "Parameters")
        #rot = cv2.getTrackbarPos("Rotation", "Parameters")/10
        img = rotate_image(img, 0)
        padx = 0
        pady = 0
        images[y][x] = img[pady:pady+(height - (2*pady)), padx:(padx + (width - (2*padx)))]
imagerows = []
for imgrow in images:
    imgrow.reverse()
    rowimg = np.concatenate(imgrow, axis=0)
    imagerows.append(rowimg)
imagerows.reverse()
cv2.imwrite("out.jpg", np.concatenate(imagerows, axis=1))
    #img1 = rotate_image(img1, -0.2)
    #img2 = rotate_image(img2, -0.2)
    #img3 = rotate_image(img3, -0.2)
    #img4 = rotate_image(img4, -0.2)
    #padx = 6
    #pady = 10
    #img1 = img1[pady:pady+(height - (2*pady)), padx:(padx + (width - (2*padx)))]
    #img2 = img2[pady:pady+(height - (2*pady)), padx:(padx + (width - (2*padx)))]
    #img3 = img3[pady:pady+(height - (2*pady)), padx:(padx + (width - (2*padx)))]
    #img4 = img4[pady:pady+(height - (2*pady)), padx:(padx + (width - (2*padx)))]
    
    #rowimg = np.concatenate([img4, img3, img2, img1], axis=0)
    #cv2.imwrite("out.jpg",rowimg)
    

cv2.destroyAllWindows()
exit()
