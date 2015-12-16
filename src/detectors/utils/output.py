#!/usr/bin/env python3
"""Contains classes and functions that generate details that will be used for localization
and tracking of object"""
import cv2

class Output(object):
    """Stores details after running object detection"""
    def __init__(self, cvimg):
        #Input image details
        self._y, self._x = cvimg.shape[:2]
        self.center = (self._x/2, self._y/2)                        
        self.total_area = self._x*self._y

        #Output image details
        self._centroid = False
        self.dx = None
        self.dy = None
        self._area_ratio = None
        self.angle = None                               #angle from major axis of object
        self.cnts = None                                #outermost contour of object
        self.outimg = cvimg

    @property
    def centroid(self):
        return self._centroid

    @centroid.setter
    def centroid(self, value):
        self._centroid = value
        self.dx = round((value[0] - self.center[0])/float(self._x), 3)     #calculates x-differential
        self.dy = round((self.center[1] - value[1])/float(self._y), 3)     #calculates y-differential

    @property
    def area_ratio(self):
        return self._area_ratio

    @area_ratio.setter
    def area_ratio(self, value):
        assert value <= self.total_area                     
        self._area_ratio = round(value/float(self.total_area), 3)
        
    @property
    def detected(self):
        return self._centroid   #object detected only when centroid is set

if __name__ == "__main__":
    output = Output(cv2.imread("../../test/resources/img/bin1.jpg"))
    print(output.detected)
