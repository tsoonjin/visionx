#!/usr/bin/env python
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage, Image
from conversion import *

img_topic = "/bottomcam/camera/image_color/compressed"

class Prototype(object):
    def __init__(self):
        rospy.init_node("prototype")
        rospy.Subscriber(img_topic, CompressedImage, self.cam_cb)

    def cam_cb(self, rosimg):
        rospy.loginfo("called")
        cvimg = readCompressed(rosimg)

if __name__ == "__main__":
    protoype = Prototype()
    rospy.spin()
        

