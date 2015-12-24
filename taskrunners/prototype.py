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

'''Finlayson Color Constant Object Recognition'''
def double_opp(chan1, chan2):
    ratio = np.divide(np.float32(chan1), np.float32(chan2))
    opp_invariant = np.nan_to_num(np.log(ratio))
    blur = cv2.GaussianBlur(np.uint8(opp_invariant), (5,5), 0) 
    output = cv2.Laplacian(np.uint8(blur), cv2.CV_64F)
    return output

'''Weight maps
Usage:
    Weight map is normalized to [0,1]. To display them please multiply by 255 and 
    convert to BGR'''

def weight_haze(img):
    """Calculates haze veil of image
    Returns:
        y : luminance component which is grayscale
    """
    estimate = np.mean(cv2.GaussianBlur(img, (5,5), 11))/255.0
    depth = 255 - img*estimate
    y_cr_cb = cv2.cvtColor(np.uint8(depth), cv2.COLOR_BGR2YCrCb)
    y,cr,cb = cv2.split(y_cr_cb)
    output = y/255.0
    return output

def weight_salient(img):
    """Calculates saliency(visual importance) map using a channel of Lab color space
    Warning:
        numpy mean, sum is much slower than regular arithmetic operation
    """
    L,a,b = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2Lab))
    mean_vector = np.divide((np.float32(L) + np.float32(a) + np.float32(b)), 3)
    output = np.linalg.norm([mean_vector - cv2.GaussianBlur(a, (3,3), 0)], axis=0)
    return output/255.0
    

def weight_exposedness(img):
    """Calculates exposedness map by modelling a Gaussian with standard deviation of 0.25
    Adapted to use hue channel and inverse it to obtain better result
    """
    h,s,l = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HLS))
    output = np.exp(-(np.power(np.float32(h)/255.0 - 0.5, 2))/0.125)
    output = (255 - output*255)/255.0
    output = output.clip(max=1)
    return output

def normalize_weights(weight_maps):
    total_weights = np.sum(weight_maps, axis=0)
    return [np.divide(x, total_weights) for x in weight_maps]
     
def enhance_by_fusion(img):
    b,g,r = cv2.split(img)
    w_salient = weight_salient(img)
    w_haze = weight_haze(img)
    w_exposedness = weight_exposedness(img)
    norm_weights = normalize_weights([w_salient, w_haze, w_exposedness])
    b = b*(norm_weights[1]+norm_weights[0])
    g = g*(norm_weights[1]+norm_weights[0])
    r = r*(norm_weights[1]+norm_weights[0])
    return cv2.merge((np.uint8(b), np.uint8(g), np.uint8(r)))

class Prototype(object):

    def __init__(self):
        signal.signal(signal.SIGINT, self.handleInterrupt)
        rospy.init_node('prototype')
        self.img_topic = rospy.get_param('~topic', 'frontcam')
        self.pub_topic = '/prototype/compressed'
        self.imgs = deque()

        '''ROS Initialization'''
        rospy.Subscriber('/{}/camera/image_color/compressed'.format(self.img_topic), 
                CompressedImage, self.cam_cb)
        self.processed = rospy.Publisher(self.pub_topic, CompressedImage)


    def detectFrame(self, cvimg):
        output = cvimg
        if len(self.imgs) == 4:
            output = motionDeflicker(self.imgs)
        return output

    def detect(self, cvimg):
        b,g,r = cv2.split(cvimg)
        output = enhance_by_fusion(cvimg)
        return output

    def handleInterrupt(self, signal, frame):
        rospy.signal_shutdown("Preeempted")
        
    def addImg(self, cvimg):
        self.imgs.append(cvimg)
        if len(self.imgs) > 4:
            self.imgs.popleft()

    '''Callbacks''' 
    def cam_cb(self, rosimg):
        cvimg = resize(readCompressed(rosimg), 3.0)   #Scale down original image by 3
        self.addImg(cvimg)
        output = writeCompressed(self.detect(cvimg))
        self.processed.publish(output)

if __name__ == "__main__":
    prototype = Prototype()
    rospy.spin()
