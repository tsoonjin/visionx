#!/usr/bin/env python3

@staticmethod
def analyzeChan(chan):
    outImg = VUtil.toBGR(chan, 'gray')
    min,mean,mid,max = (np.min(chan),np.mean(chan),np.median(chan),np.max(chan))
    var,std = (np.var(chan), np.std(chan))
    median_img = cv2.threshold(chan, mid, 255, cv2.THRESH_BINARY)[1]
    mean_img = cv2.threshold(chan, 90, 255, cv2.THRESH_BINARY)[1]
    print("Min:%f, Mean:%f, Median:%f, Max:%f"%(min,mean,mid,max))
    print("Var:%f, Std:%f"%(var,std))
    return VUtil.toBGR(chan, 'gray')

@staticmethod
def analyzeSalient(chan):
    empty = np.ones_like(chan)
    mean = np.mean(chan)
    mean = empty*mean
    blur = cv2.GaussianBlur(chan, (21,21),1)
    final = mean - blur
    final = final.clip(min=0)
    final = np.uint8(final)
    return np.std(final)

@staticmethod
def analyze(img):
    hsv = VUtil.getHSV(img)
    bgr = VUtil.getRGB(img)
    luv = VUtil.getLUV(img)
    lab = VUtil.getLAB(img)
    return np.vstack((bgr,hsv,luv,lab))

@staticmethod 
def getRGB(img):
    b,g,r = cv2.split(img)
    b = cv2.cvtColor(b, cv2.COLOR_GRAY2BGR)
    g = cv2.cvtColor(g, cv2.COLOR_GRAY2BGR)
    r = cv2.cvtColor(r, cv2.COLOR_GRAY2BGR)
    return np.hstack((b,g,r))

@staticmethod 
def getHSV(img):
    return VUtil.getRGB(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))

@staticmethod 
def getLUV(img):
    return VUtil.getRGB(cv2.cvtColor(img, cv2.COLOR_BGR2LUV))

@staticmethod 
def getYCB(img):
    return VUtil.getRGB(cv2.cvtColor(img, cv2.COLOR_BGR2YCR_CB))

@staticmethod 
def getLAB(img):
    return VUtil.getRGB(cv2.cvtColor(img, cv2.COLOR_BGR2LAB))

@staticmethod
def bgrstd(img):
    b,g,r = cv2.split(img)
    std = np.std([b,g,r])
    return  std

@staticmethod
def labvar(img):
    l,a,b = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2LAB))
    var = np.var([l,a,b])
    return  var

@staticmethod
def vc(img):
    h,l,s = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HLS))
    roughness = np.std(l)/np.mean(l)
    return roughness

@staticmethod
def channelThresh(img,block=51,offset=5):
    a,b,c = cv2.split(img)
    a = cv2.adaptiveThreshold(a, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, block,offset)
    b = cv2.adaptiveThreshold(b, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, block,offset)
    c = cv2.adaptiveThreshold(c, 255, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, block,offset)
    return np.hstack((VUtil.toBGR(a,'gray'),VUtil.toBGR(b,'gray'),VUtil.toBGR(c,'gray')))

@staticmethod
def channelThreshColor(img):
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
    a = VUtil.channelThresh(img)
    b = VUtil.channelThresh(hsv)
    c = VUtil.channelThresh(lab)
    return np.vstack((a,b,c))

