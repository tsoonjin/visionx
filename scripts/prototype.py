#!/usr/bin/env python
import signal
from collections import deque
from timeit import timeit

import rospy
import cv2
import numpy as np 
from numba import autojit
from scipy.ndimage.filters import minimum_filter,uniform_filter
from sensor_msgs.msg import CompressedImage

from vision_lib.conversion import *
from vision_lib.stats import *


def calcTransmissionMap(img,airlight):
    A_b, A_g, A_r = airlight
    A_r = 255 - A_r + 0.001   #Prevent division by zero
    out = np.empty_like(img, dtype=np.float32)
    row, col = img.shape[:2]
    for x in xrange(row):
        for y in xrange(col):
            x_init = max(x-1, 0)
            x_final = min(x+2, row-1)
            y_init = max(y-1,0)
            y_final = min(y+2, col-1)
            out[x,y,0] = np.min(img[x_init:x_final, y_init:y_final,0])/A_b
            out[x,y,1] = np.min(img[x_init:x_final, y_init:y_final,1])/A_g
            out[x,y,2] = np.min(255 - img[x_init:x_final, y_init:y_final,2])/A_r
    out =  np.max(1 - np.min(out, axis=2), 0.1) #0.1 is the minimum transmission map value
    return out

tMap = autojit(calcTransmissionMap)

def waterlight_estimation(img):
    """Estimate brighest pixel in the image"""
    img = np.float32(img)
    b,g,r = cv2.split(np.float32(img))
    minval,maxval,minloc,maxloc = cv2.minMaxLoc(r)
    waterlight = img[maxloc[1], maxloc[0]]
    return waterlight

def redchannelprior(img):
    airlight = waterlight_estimation(img)
    t = tMap(img, airlight)
    return img

def minkowski_norm(grayimg, p):
    white_grayimg = np.power(np.sum(np.power(grayimg, p)), 1.0/p)
    return white_grayimg

def color_preprocess(img):
    """Remove clipped pixels, median filter and resize"""
    img = cv2.medianBlur(img, 5)
    clahe = cv2.createCLAHE(clipLimit=2.0)
    b,g,r = cv2.split(img)
    b = clahe.apply(b)
    g = clahe.apply(g)
    r = clahe.apply(r)
    return cv2.merge((b,g,r))

def grayworld_norm2(img):
    b,g,r = cv2.split(img)
    est = np.mean([b,g,r])
    b = np.divide(est, np.mean(b))*b
    b = b.clip(max=255)
    g = np.divide(est, np.mean(g))*g
    g = g.clip(max=255)
    r = np.divide(est, np.mean(r))*r
    r = r.clip(max=255)
    return cv2.merge((np.uint8(b), np.uint8(g), np.uint8(r)))

def grayworld_norm1(img):
    b,g,r = cv2.split(img)
    est = np.max([np.mean(b), np.mean(g), np.mean(r)])
    b = est/np.mean(b)*b
    b = b.clip(max=255)
    g = est/np.mean(g)*g
    g = g.clip(max=255)
    r = est/np.mean(r)*r
    r = r.clip(max=255)
    return cv2.merge((np.uint8(b), np.uint8(g), np.uint8(r)))

def max_rgb(img):
    """Based on Land's retinex theory"""
    b,g,r = cv2.split(img)
    b = np.float32(b)/np.max(b)*b
    g = np.float32(g)/np.max(g)*g
    r = np.float32(r)/np.max(r)*r
    return cv2.merge((np.uint8(b), np.uint8(g), np.uint8(r)))

def log_normalization(img):
    """Non-iterative comprehensive normalization of color
    Only returns a descriptor cannot be displayed correctly
    """
    b,g,r = cv2.split(img)
    b = np.nan_to_num(np.log(np.float32(b)))
    mean_b = np.mean(b)
    g = np.nan_to_num(np.log(np.float32(g)))
    mean_g = np.mean(g)
    r = np.nan_to_num(np.log(np.float32(r)))
    mean_r = np.mean(r)
    mean_bgr = (b+g+r)/3
    b = np.exp(b - mean_b - mean_bgr)
    g = np.exp(g - mean_g - mean_bgr)
    r = np.exp(r - mean_r - mean_bgr)
    output = img
    return output

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
    Notes:
        Should apply only to blue and green channels
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
    return (output/np.max(output))
    

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
        output = cvimg
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
