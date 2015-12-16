#!/usr/bin/env python
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

def readCompressed(rosimg):
    np_arr = np.fromstring(rosimg.data, np.uint8)
    return cv2.imdecode(np_arr, cv2.IMREAD_ANYCOLOR)

def writeCompressed(img):
    msg = CompressedImage()
    msg.header.stamp = rospy.Time.now()
    msg.format = "jpeg"
    msg.data = np.array(cv2.imencode('.jpeg', img)[1]).tostring()
    return msg

def sk_to_cv(img):
    return cv2.cvtColor(img, cv2.COLOR_RGB2BGR)

def cv_to_sk(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

def rosimg2cv(ros_img):
    try:
        frame = bridge.imgmsg_to_cv2(ros_img, desired_encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)

    return frame

def cv2rosimg(cv_img):
    try:
        return bridge.cv2_to_imgmsg(cv_img, encoding="bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)

def toBGR(img, flag):
    if flag is 'gray':
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if flag is 'hsv':
        return cv2.cvtColor(img, cv2.COLOR_HSV2BGR)
    if flag is 'lab':
        return cv2.cvtColor(img, cv2.COLOR_LAB2BGR)

def toHSV(img):
    """Converts BGR to HSV"""
    return cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

def toLAB(img):
    """Converts BGR to LAB"""
    return cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
