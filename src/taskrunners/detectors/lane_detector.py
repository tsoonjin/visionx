#!/usr/bin/env python3
import math

import cv2
import numpy as np

from utils.output import Output
from utils.enhancement import *
from utils.feature import *
from utils.draw import *
from utils.stats import *

class LaneDetector(object):
    def __init__(self):
        self.processed = {}

    def color_correct(self, img):
        self.processed['color_corrected'] = finlaynorm(iace(img))
        return self.processed['color_corrected']

    def threshold(self, img):
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

    def findContour(self, threshImg, output):
        #Only retrieves contour from return value [image, contours, hieararchy]
        contours = cv2.findContours(threshImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[1]
        contours = sorted(contours, key=cv2.contourArea, reverse=True)
        selected = []   #Select contour that fits criteria
        if len(contours) >= 1:

            for cnt in contours:
                rect = cv2.minAreaRect(cnt)
                points = np.int32(cv2.boxPoints(rect))
                edge1 = points[1] - points[0]
                edge2 = points[2] - points[1]
                #Remove false positive by limiting area
                cnt = cv2.convexHull(cnt)
                if cv2.contourArea(cnt) > 700 and checkRectangle(cnt):
                    selected.append(np.int0(cv2.boxPoints(rect)))
                    if cv2.norm(edge1) > cv2.norm(edge2):
                        rectAngle = math.degrees(math.atan2(edge1[1], edge1[0]))
                    else:
                        rectAngle = math.degrees(math.atan2(edge2[1], edge2[0]))
                    output.centroid = get_centroid(cnt)
                    output.angle = 90 - abs(rectAngle) if rectAngle >= -90 else 90 - abs(rectAngle)
                    break
        output.contours = selected

    def detect(self, cvimg):
        output = Output(cvimg)
        self.processed['input'] = cvimg
        color_corrected = self.color_correct(cvimg)
        threshImg = self.threshold(color_corrected)
        self.findContour(threshImg, output)
        output.processed = self.processed
        return output


if __name__ == "__main__":
    img_path = 'lane1'
    cvimg = cv2.imread('./resources/img/{}.jpg'.format(img_path))
    laneDetector = LaneDetector()
    output = laneDetector.detect(cvimg)
    cv2.imshow('test', output)
    cv2.waitKey(0)
