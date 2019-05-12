from __future__ import print_function
from imutils.object_detection import non_max_suppression
import numpy as np
import cv2


def detectPedestrian(image, winStride, scale):
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    # detect people in the image
    (rects, weights) = hog.detectMultiScale(image, winStride=(winStride, winStride),
                                            padding=(8, 8), scale=scale)

    # apply non-maxima suppression to the bounding boxes using a
    # fairly large overlap threshold to try to maintain overlapping
    # boxes that are still people
    rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
    pick = non_max_suppression(rects, probs=None, overlapThresh=0.5)

    return pick
