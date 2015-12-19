#!/usr/bin/env python
import signal
from collections import deque

import rospy
import cv2
import numpy as np 
from sensor_msgs.msg import CompressedImage

from configs.base_config import BaseConfig
from rosbridge.base_comm import BaseComm
from detectors.utils.conversion import *
from detectors.utils.stats import *

class Prototype(object):

    def __init__(self):
        signal.signal(signal.SIGINT, self.handleInterrupt)
        rospy.init_node('prototype')
        self.img_topic = rospy.get_param('~topic', 'bottomcam')
        self.pub_topic = '/prototype/compressed'
        self.imgs = deque()

        '''ROS Initialization'''
        rospy.Subscriber('/{}/camera/image_color/compressed'.format(self.img_topic), 
                CompressedImage, self.cam_cb)
        self.processed = rospy.Publisher(self.pub_topic, CompressedImage)

    def detect(self, cvimg):
        output = cvimg
        return output


    def handleInterrupt(self, signal, frame):
        rospy.signal_shutdown("Preeempted")
        
    '''Callbacks''' 
    def cam_cb(self, rosimg):
        cvimg = resize(readCompressed(rosimg), 3.0)   #Scale down original image by 3
        output = writeCompressed(self.detect(cvimg))
        self.processed.publish(output)

if __name__ == "__main__":
    prototype = Prototype()
    rospy.spin()
