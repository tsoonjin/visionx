#!/usr/bin/env python
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

import cv2
import numpy as np

import sys

bridge = CvBridge()
loThresh = np.array([0, 0, 0])
hiThresh = np.array([255, 255, 255])

mode = 'bgr8'

def rosimg2cv(ros_img):
    global bridge
    try:
        frame = bridge.imgmsg_to_cv2(ros_img, desired_encoding=mode)
    except CvBridgeError as e:
        rospy.logerr(e)

    return frame

bins = np.arange(256).reshape(256,1)

def hist_curve(im):
    h = np.zeros((300,256,3))
    if len(im.shape) == 2:
        color = [(255,255,255)]
    elif im.shape[2] == 3:
        color = [ (255,0,0),(0,255,0),(0,0,255) ]
    for ch, col in enumerate(color):
        hist_item = cv2.calcHist([im],[ch],None,[256],[0,256])
        cv2.normalize(hist_item,hist_item,0,255,cv2.NORM_MINMAX)
        hist=np.int32(np.around(hist_item))
        pts = np.int32(np.column_stack((bins,hist)))
        cv2.polylines(h,[pts],False,col)
    y=np.flipud(h)
    return y

def hist_lines(im, mask=None, color=(255, 255, 255), lower=0, upper=255):
    h = np.zeros((300,256,3))
    if len(im.shape)!=2:
        print "hist_lines applicable only for grayscale images"
        im = cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)
    if mask is None:
        hist_item = cv2.calcHist([im],[0],None,[256],[0,256])
    else:
        hist_item = cv2.calcHist([im], [0], mask, [256], [0,256])
    cv2.normalize(hist_item,hist_item,0,255,cv2.NORM_MINMAX)
    hist=np.int32(np.around(hist_item))
    for x,y in enumerate(hist):
        cv2.line(h,(x,0),(x,y),color)
    cv2.line(h, (lower,0), (lower,300), color)
    cv2.line(h, (upper,0), (upper,300), color)
    y = np.flipud(h)
    return y


def onChange(data):
    global loThresh, hiThresh
    loThresh[0] = cv2.getTrackbarPos("loH", "output")
    hiThresh[0] = cv2.getTrackbarPos("hiH", "output")
    loThresh[1] = cv2.getTrackbarPos("loS", "output")
    hiThresh[1] = cv2.getTrackbarPos("hiS", "output")
    loThresh[2] = cv2.getTrackbarPos("loV", "output")
    hiThresh[2] = cv2.getTrackbarPos("hiV", "output")


def shadesofGray(img):
    inB, inG, inR = cv2.split(img)
    avgR = np.mean(inR)
    avgG = np.mean(inG)
    avgB = np.mean(inB)
    avgGray = np.mean((avgB, avgG, avgR))

    if avgB == 0:
        outB = inB
    else:
        outB = (avgGray/avgB)*inB

    if avgG == 0:
        outG = inG
    else:
        outG = (avgGray/avgG)*inG

    if avgR == 0:
        outR = inR
    else:
        outR = (avgGray/avgR)*inR

    outImg = cv2.merge((np.uint8(outB), np.uint8(outG), np.uint8(outR)))
    outImg = cv2.normalize(outImg, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX)
    return outImg


def illuminanceMask(img):
    grayImg = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    grayImg = cv2.equalizeHist(grayImg)
    return cv2.threshold(grayImg, 200, 255, cv2.THRESH_BINARY)[1]


def enhance(img):
    blurImg = cv2.GaussianBlur(img, ksize=(0, 0), sigmaX=10)
    enhancedImg = cv2.addWeighted(img, 2.5, blurImg, -1.5, 0)
    return enhancedImg

def whiteBalance(img):
    b, g, r = cv2.split(img)
    b = cv2.equalizeHist(b)
    g = cv2.equalizeHist(g)

    img = cv2.merge((b, g, r))
    return img

def makeTable(div):
    table = [i for i in range(256)]
    table = np.uint8(map(lambda i: i/div*div + div/2, table))
    return table


table = makeTable(100)
def reduceVal(img):
    global table
    return cv2.LUT(img, table)

def drawRect(img, pts, color=(0, 0, 255)):
    points = np.int32(pts)
    for i in range(4):
        pt1 = (points[i][0], points[i][1])
        pt2 = (points[(i+1) % 4][0], points[(i+1) % 4][1])
        cv2.line(img, pt1, pt2, color, 2)

processingCount = 0
colors = [ (255,0,0),(0,255,0),(0,0,255) ]
maskPoints = list()

def processSonar(rawImg):
    processedImg = rawImg.copy()
    processedImg = cv2.equalizeHist(processedImg)
    processedImg = cv2.GaussianBlur(processedImg, (9, 9), 0)
    maxIntensity = np.max(processedImg)
    _, processedImg = cv2.threshold(processedImg, maxIntensity - 10, 255, cv2.THRESH_BINARY)
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
    processedImg = cv2.morphologyEx(processedImg, cv2.MORPH_OPEN, kernel, iterations=1)
    processedImg.shape = (processedImg.shape[0], processedImg.shape[1], 1)
    return np.hstack((rawImg, processedImg))

def camCallback(rosImg):
    global loThresh, hiThresh, processingCount

    rawImg = rosimg2cv(rosImg)

    if mode == "bgr8":
        rawImg = cv2.resize(rawImg, (320, 250))
        rawImg = enhance(rawImg)
        rawImg = cv2.GaussianBlur(rawImg, (5,5), 0)

        hsvImg = cv2.cvtColor(rawImg, cv2.COLOR_BGR2HSV)
        threshImg = cv2.inRange(hsvImg, loThresh, hiThresh)
        threshImg = cv2.cvtColor(threshImg, cv2.COLOR_GRAY2BGR)

        mask = None
        if len(maskPoints) > 0:
            mask = np.zeros(rawImg.shape[:2], dtype=np.uint8)
            cv2.fillPoly(mask, np.int32([maskPoints]), 255)
            cv2.polylines(rawImg, [np.int32(maskPoints)], True, (0, 255, 255), 2)

        hsvSplit = cv2.split(hsvImg)
        histImgs = list()
        for i,c in enumerate(hsvSplit):
            histImgs.append(hist_lines(c, mask, colors[i], loThresh[i], hiThresh[i]))

        if mask is None:
            outImg = np.hstack((rawImg, threshImg))
        else:
            outImg = np.hstack((rawImg, threshImg))

        cv2.imshow("hist", np.hstack(histImgs))
        cv2.imshow("output", outImg)
    elif mode == "mono8":
        cv2.imshow("output", processSonar(rawImg))
    cv2.waitKey(5)
    processingCount = 0


def mouseCb(event, x, y, flags, param):
    global maskPoints
    if event == cv2.EVENT_LBUTTONDOWN:
        maskPoints.append((x,y))
    elif event == cv2.EVENT_RBUTTONDOWN:
        maskPoints = list()


def main():
    global mode
    img_topic = "/bot_camera/camera/image_raw_jin"
    img_topic = img_topic if len(sys.argv) < 2 else sys.argv[1]
    if img_topic == "/sonar_image" or img_topic == "/sonar_image_thien":
        mode = "mono8"

    cv2.namedWindow("output")
    cv2.createTrackbar("loH", "output", 0, 180, onChange)
    cv2.createTrackbar("hiH", "output", 0, 180, onChange)
    cv2.createTrackbar("loS", "output", 0, 255, onChange)
    cv2.createTrackbar("hiS", "output", 0, 255, onChange)
    cv2.createTrackbar("loV", "output", 0, 255, onChange)
    cv2.createTrackbar("hiV", "output", 0, 255, onChange)

    cv2.setMouseCallback("output", mouseCb)

    rospy.Subscriber(img_topic, Image, camCallback)

    rospy.spin()

if __name__ == "__main__":
    rospy.init_node("hsv_threshold")
    main()
