import numpy as np
import cv2, os, PIL.Image

here = os.path.dirname(os.path.abspath(__file__))

def cropVisiblePart(img): # crops the part of the image that may contain reflections
    ylength, xlength, c = img.shape
    x1 = 0
    y1 = 0
    if xlength > 0 and ylength > 0 and ((y1 + int(ylength / 5)) > y1) and x1 + xlength > x1:
        return img[0:ylength,
               int(x1 + xlength / 1.8): int(x1 + xlength)]
    else:
        return []

# these values have been calculated with ./calib.py
# for reference see
# https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_calib3d/py_calibration/py_calibration.html
DIM=(729, 1232)
K=np.array([[65949.79058215416, 0.0, -743.2075275257644], [0.0, 65780.92244287752, 685.2983318216144], [0.0, 0.0, 1.0]])
D=np.array([[7.21225896261792], [-156752.02943681442], [186651207.96973005], [-70768024746.31644]])
map1, map2 = cv2.fisheye.initUndistortRectifyMap(K, D, np.eye(3), K, DIM, cv2.CV_16SC2)

def rotate_image(image, angle): # corrects rotational error of camera.
  image_center = tuple(np.array(image.shape[1::-1]) / 2)
  rot_mat = cv2.getRotationMatrix2D(image_center, angle, 1.0)
  result = cv2.warpAffine(image, rot_mat, image.shape[1::-1], flags=cv2.INTER_LINEAR)
  return result

def unwarpImage(img): # corrects fisheye effect
    (height, width) = img.shape[:2]
    img = cv2.remap(img, map1, map2, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT)
    img = rotate_image(img, 1615.5)
    img = img[26:26+(height - (2*26)), 54:(54 + (width - (2*54)))]
    return img
    