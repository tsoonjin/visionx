#!/usr/bin/env python3
import rospy
import cv2
import numpy as np

from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()

#====================================
# Handles conversion of image formats
#====================================

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

#=======================================
# Image statistics and utility functions
#=======================================

def checkRectangle(cnt,ratio_limit=1.2):
    rect = cv2.minAreaRect(cnt)
    rect_area = rect[1][0]*rect[1][1]
    ratio = rect_area/(cv2.contourArea(cnt)+0.001)
    return ratio < ratio_limit

def checkCircle(cnt):
    circle = cv2.minEnclosingCircle(cnt)
    circle_area = (circle[1]**2)*math.pi
    ratio = circle_area/(cv2.contourArea(cnt)+0.001)
    return ratio < 1.5

def rectify(img,rect):
    h = np.int0(np.around(cv2.cv.BoxPoints(rect)))
    h = h.reshape((4,2))
    hnew = np.zeros((4,2),dtype = np.float32)
    add = h.sum(1)
    hnew[0] = h[np.argmin(add)]
    hnew[2] = h[np.argmax(add)]
    diff = np.diff(h,axis = 1)
    hnew[1] = h[np.argmin(diff)]
    hnew[3] = h[np.argmax(diff)]
    return img[hnew[0][1]:hnew[2][1],hnew[0][0]:hnew[2][0]]

def z_norm(arr):
    minX = np.min(arr)
    maxX = np.max(arr)
    return arr-minX/(maxX-minX)

def blockiter(img, func, blksize=(30,30)):
    mask = np.zeros(img.shape[:2], dtype=np.float32)
    y,x = img.shape[:2]
    for i in xrange(0,y,5):
        dy = blksize[1]-1 if i+blksize[1] < y else y-i-1
        for j in xrange(0,x,5):
            dx = blksize[0]-1 if j+blksize[0] < x else x-j-1
            view = img[i:i+dy,j:j+dx]
            mask[i:i+dy,j:j+dx] = func(view)
    return mask

def calc_hist(src):
    h = np.zeros((src.shape[0],256,3))
    bins = np.arange(256).reshape(256,1)
    color = [ (255,0,0),(0,255,0),(0,0,255) ]

    for ch, col in enumerate(color):
        hist_item = cv2.calcHist([src],[ch],None,[256],[0,255])
        cv2.normalize(hist_item,hist_item,0,255,cv2.NORM_MINMAX)
        hist=np.int32(np.around(hist_item))
        pts = np.column_stack((bins,hist))
        cv2.polylines(h,[pts],False,col)
    h=np.flipud(h)
    return np.uint8(h)

def resize(img, scale=2.0):
    scale = 1/scale
    return cv2.resize(img, (int(img.shape[1]*scale), int(img.shape[0]*scale)))

def blend(img1, a, img2, b, g):
    return cv2.addWeighted(img1, a, img2, b, g)

def genGPyramid(img, level=6):
    """Generate Gaussian Pyramid of 6 levels"""
    G = img.copy()
    gp = [img]
    for i in xrange(level):
        G = cv2.pyrDown(G)
        gp.append(G)
    return gp

def genLPyramid(gp):
    """Generate Laplacian Pyramid"""
    size = len(gp) - 2 
    lp = [gp[size]]
    for i in xrange(size, 0, -1):
        blur = gp[i-1]
        GE = cv2.pyrUp(gp[i])
        GE = cv2.resize(GE, (blur.shape[1], blur.shape[0]))
        L = cv2.subtract(blur, GE)
        lp.append(L)
    return lp

def hist_info(chan):#For iace
    done_low = True
    hist,bins = np.histogram(chan,256,[0,256])
    cdf = hist.cumsum()
    low = int(chan.size*0.04)
    hi = int(chan.size*0.96)
    for h,i in enumerate(cdf):
        if i > low and done_low:
            low_thresh = h
            done_low = False
        if i > hi:
            high_thresh = h
            break
    return (low_thresh, high_thresh)
        
def getHist(chan,color=(0,0,255)):#For iace
    hist = cv2.calcHist([chan],[0],None,[256],[0,256])
    mask = np.zeros((300,256,3))
    for x,y in enumerate(np.int32(np.around(hist))):
        cv2.line(mask, (x,299), (x,y), color)
    return np.uint8(mask)

def integral(img):
    res = cv2.integral(img)
    return img

def confidenceMask(img,offset=0.5):
    cent_x = img.shape[1]/2.0
    cent_y = img.shape[0]/2.0
    mask = np.zeros_like(img)
    cv2.circle(mask, (int(cent_x),int(cent_y)), int(offset*cent_y*2), (255,255,255), -1) 
    return cv2.cvtColor(mask, cv2.COLOR_BGR2GRAY)

def euclid_dist(ls):
    sum = 0
    for x,y in ls:
        sum += np.power(x-y,2)
    return np.sqrt(sum)

def analyzeChan(chan):
    outImg = VUtil.toBGR(chan, 'gray')
    min,mean,mid,max = (np.min(chan),np.mean(chan),np.median(chan),np.max(chan))
    var,std = (np.var(chan), np.std(chan))
    median_img = cv2.threshold(chan, mid, 255, cv2.THRESH_BINARY)[1]
    mean_img = cv2.threshold(chan, 90, 255, cv2.THRESH_BINARY)[1]
    print("Min:%f, Mean:%f, Median:%f, Max:%f"%(min,mean,mid,max))
    print("Var:%f, Std:%f"%(var,std))
    return VUtil.toBGR(chan, 'gray')

def analyzeSalient(chan):
    empty = np.ones_like(chan)
    mean = np.mean(chan)
    mean = empty*mean
    blur = cv2.GaussianBlur(chan, (21,21),1)
    final = mean - blur
    final = final.clip(min=0)
    final = np.uint8(final)
    return np.std(final)

def analyze(img):
    hsv = VUtil.getHSV(img)
    bgr = VUtil.getRGB(img)
    luv = VUtil.getLUV(img)
    lab = VUtil.getLAB(img)
    return np.vstack((bgr,hsv,luv,lab))

def getRGB(img):
    b,g,r = cv2.split(img)
    b = cv2.cvtColor(b, cv2.COLOR_GRAY2BGR)
    g = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    r = cv2.cvtColor(r, cv2.COLOR_GRAY2BGR)
    return np.hstack((b,g,r))

def getHSV(img):
    return VUtil.getRGB(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))

def getLUV(img):
    return VUtil.getRGB(cv2.cvtColor(img, cv2.COLOR_BGR2LUV))

def getYCB(img):
    return VUtil.getRGB(cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB))

def getLAB(img):
    return VUtil.getRGB(cv2.cvtColor(img, cv2.COLOR_BGR2LAB))

def bgrstd(img):
    b,g,r = cv2.split(img)
    std = np.std([b,g,r])
    return  std

def labvar(img):
    l,a,b = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2LAB))
    var = np.var([l,a,b])
    return  var

def vc(img):
    h,l,s = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HLS))
    roughness = np.std(l)/np.mean(l)
    return roughness

def channelThresh(img,block=51,offset=5):
    a,b,c = cv2.split(img)
    a = cv2.adaptiveThreshold(a, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, block,offset)
    b = cv2.adaptiveThreshold(b, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, block,offset)
    c = cv2.adaptiveThreshold(c, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, block,offset)
    return np.hstack((VUtil.toBGR(a,'gray'),VUtil.toBGR(b,'gray'),VUtil.toBGR(c,'gray')))

def channelThreshColor(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    a = VUtil.channelThresh(img)
    b = VUtil.channelThresh(hsv)
    c = VUtil.channelThresh(lab)
    return np.vstack((a,b,c))
