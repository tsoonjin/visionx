#!/usr/bin/env python3
import cv2
import numpy as np

from utils.enhancement import *

class LaneDetector(object):
    def __init__(self):
        self.processed = {}

    def color_correct(self, img):
        self.processed['color_corrected'] = finlaynorm(iace(img))
        return self.processed['color_corrected']

    def threshold(img):
        b,g,r = cv2.split(img)
        h,s,v = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))

        #Threshold range
        lowerRed1 = (0,0,150)
        upperRed1 = (20,255,255)
        lowerRed2 = (170,0,150)
        upperRed2 = (180,255,255)

        #Generating 2 masks because of the range of red color at both ends of the spectrum
        mask1 = cv2.inRange(cv2.cvtColor(img,cv2.COLOR_BGR2HSV), lowerRed1, upperRed1)
        mask2 = cv2.inRange(cv2.cvtColor(img,cv2.COLOR_BGR2HSV), lowerRed2, upperRed2)
        orange = mask1| mask2

        #Single blue channel threholding
        common = cv2.threshold(b, 40, 255, cv2.THRESH_BINARY_INV)[1]

        output = common & orange
        self.processed['thresh'] = cv2.cvtColor(output, cv2.COLOR_GRAY2BGR)
        return output

    def detect(cvimg):
        color_corrected = self.color_correct(cvimg)
        threshImg = self.threshold(color_corrected)
        output = cv2.cvtColor(threshImg, cv2.COLOR_GRAY2BGR)
        return output


if __name__ == "__main__":
    img_path = 'lane1'
    cvimg = cv2.imread('./resources/img/{}.jpg'.format(img_path))
    output = detect(cvimg)
    cv2.imshow('test', output)
    cv2.waitKey(0)
