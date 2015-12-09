#!/usr/bin/env python3

@staticmethod 
def DoG(img, kern1=(3,3), kern2=(5,5)):
    """Difference of Gaussian using diff kernel size"""
    smooth1 = cv2.GaussianBlur(img, kern1,0)
    smooth2 = cv2.GaussianBlur(img, kern2,0)
    final = smooth1 - smooth2
    return final

@staticmethod
def normIllumColor(img,gamma=2.2):
    img = np.float32(img)
    img /= 255.0
    img = cv2.pow(img, 1/gamma)*255
    img = np.uint8(img)
    return img

@staticmethod
def gamma_correct(img):
    gamma = 2.2
    inverse_gamma = 1.0/gamma
    b,g,r = cv2.split(img)
    b = np.uint8(cv2.pow(b/255.0, inverse_gamma)*255)
    g = np.uint8(cv2.pow(g/255.0, inverse_gamma)*255)
    r = np.uint8(cv2.pow(r/255.0, inverse_gamma)*255)
    return cv2.merge((b,g,r))

@staticmethod
def sharpen(img):
    blur = cv2.GaussianBlur(img, (5,5), 5)
    res = cv2.addWeighted(img, 1.5, blur, -0.5, 0)
    return res

@staticmethod
def deilluminate(img):
    h,s,gray = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
    blur = cv2.GaussianBlur(gray, (63,63), 41)
    gray = cv2.log(np.float32(gray))
    blur = cv2.log(np.float32(blur))
    res = np.exp(gray-blur)
    res = cv2.normalize(res, 0, 255, cv2.NORM_MINMAX)*255
    v = np.uint8(res)
    return cv2.cvtColor(cv2.merge((h,s,v)), cv2.COLOR_HSV2BGR)

@staticmethod
def homomorphic(img):
    h,s,gray = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
    gray = cv2.log(np.float32(gray))
    blur = cv2.GaussianBlur(gray, (63,63), 41)
    res = np.exp(gray-blur)
    res = cv2.normalize(res, 0, 255, cv2.NORM_MINMAX)*255
    v = np.uint8(res)
    return cv2.cvtColor(cv2.merge((h,s,v)), cv2.COLOR_HSV2BGR)

@staticmethod
def deilluminate_single(gray):
    blur = cv2.GaussianBlur(gray, (63,63), 41)
    gray = cv2.log(np.float32(gray))
    blur = cv2.log(np.float32(blur))
    res = np.exp(gray-blur)
    res = cv2.normalize(res, 0, 255, cv2.NORM_MINMAX)*255
    gray = np.uint8(res)
    return gray

@staticmethod
def motiondeflicker(frames, img):
    log_median = cv2.log(np.float32(np.median(frames, axis=0)))
    log_img = cv2.log(np.float32(img))
    diff = cv2.GaussianBlur(log_img - log_median, (21,21),0)
    res = img/np.exp(diff)
    res = res.clip(max=255)
    blur = cv2.GaussianBlur(np.uint8(res), (5,5), 0)
    res = cv2.addWeighted(np.uint8(res), 1.5, blur, -0.5, 0)
    return res

@staticmethod
def deilluminate2(img):
    b,g,r = cv2.split(img)
    h,s,v = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
    log_v = cv2.log(np.float32(v))
    blur_v = cv2.log(np.float32(cv2.GaussianBlur(v, (63,63),41)))
    res = np.exp(log_v-blur_v)
    return cv2.cvtColor(np.uint8(res*255),cv2.COLOR_GRAY2BGR)

@staticmethod
def gamma1(gray):
    gray = np.float32(gray)
    gray /= 255.0
    gray = 0.3*((cv2.log(2*gray+0.1))+abs(np.log(0.1)))
    return  np.uint8(gray*255)

@staticmethod
def gamma2(gray):
    gray = np.float32(gray)
    gray /= 255.0
    gray = 0.8*(cv2.pow(gray,2))
    return  np.uint8(gray*255)

@staticmethod
def gamma3(gray):
    gray = np.float32(gray)
    gray /= 255.0
    total = 1/(np.exp(8*(gray - 0.5))+1)*255
    return np.uint8(total)

@staticmethod
def gamma1color(img):
    b,g,r = cv2.split(img)
    b = VUtil.gamma1(b)
    g = VUtil.gamma1(g)
    r = VUtil.gamma1(r)
    return cv2.merge((b,g,r))

@staticmethod
def gamma2color(img):
    b,g,r = cv2.split(img)
    b = VUtil.gamma2(b)
    g = VUtil.gamma2(g)
    r = VUtil.gamma2(r)
    return cv2.merge((b,g,r))

@staticmethod
def gamma3color(img):
    b,g,r = cv2.split(img)
    b = VUtil.gamma3(b)
    g = VUtil.gamma3(g)
    r = VUtil.gamma3(r)
    return cv2.merge((b,g,r))

"""Features"""
@staticmethod
def rg(img):
    b,g,r = cv2.split(img)
    return cv2.absdiff(r,g)

@staticmethod
def conspicuityMaps(img): 
    """Generate conspicutiy maps from intensity"""
    b,g,r = cv2.split(img)
    b = np.float32(b)
    g = np.float32(g)
    r = np.float32(r)
    intensity = np.mean(np.array([b,g,r]), axis=0)
    b /= intensity
    g /= intensity
    r /= intensity
    b = cv2.normalize(b, 0, 255, cv2.NORM_MINMAX)*255
    g = cv2.normalize(g, 0, 255, cv2.NORM_MINMAX)*255
    r = cv2.normalize(r, 0, 255, cv2.NORM_MINMAX)*255
    normBGR = cv2.merge((np.uint8(b),np.uint8(g),np.uint8(r)))
    R = r - (g+b)/2
    G = g - (r+b)/2
    B = b - (r+g)/2
    Y = (r+g)/2 - abs(r-g)/2
    Y = Y.clip(min=0)
    #out = cv2.cvtColor(np.uint8(intensity), cv2.COLOR_GRAY2BGR)
    R = cv2.cvtColor(np.uint8(R), cv2.COLOR_GRAY2BGR)
    B = cv2.cvtColor(np.uint8(B), cv2.COLOR_GRAY2BGR)
    G = cv2.cvtColor(np.uint8(G), cv2.COLOR_GRAY2BGR)
    Y = cv2.cvtColor(np.uint8(Y), cv2.COLOR_GRAY2BGR)
    return np.hstack((B,G,R,Y)) 

@staticmethod
def orb(img):
    orb = cv2.ORB_create()
	kp = orb.detect(img, None)
	kp, des = orb.compute(img, kp)
	out = None
	out = cv2.drawKeypoints(img, kp, out, color=(0,0,255), flags=0)
	return out

@staticmethod
def harris(img,block=21,aperture=11,param=0.2):
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    img = VUtil.finlaynorm(img)
    gray,g,r = cv2.split(img)
    corner = cv2.cornerHarris(np.float32(gray),block,aperture,param)
    corner = cv2.dilate(corner,None)
    img[corner>0.01*corner.max()] = (0,0,255)
    return img

@staticmethod
def fastDetector(img):
    fast = cv2.FastFeatureDetector()
    kp = fast.detect(img,None)
    img = cv2.drawKeypoints(img, kp, color=(255,0,0))
    return img

@staticmethod
def shiDetector(img):
    corners = cv2.goodFeaturesToTrack(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), 25, 0.01, 10)
    corners = np.int0(corners)
    for i in corners:
        x,y = i.ravel()
        cv2.circle(img,(x,y),3,255,-1)
    return img

@staticmethod
def briefDetector(img):
    star = cv2.FeatureDetector_create('STAR')
    brief = cv2.DescriptorExtractor_create('BRIEF')
    kp = star.detect(img,None)
    kp, desc = brief.compute(img, kp)
    img = cv2.drawKeypoints(img, kp, color=(255,0,0))
    return img

@staticmethod
def surfDetector(img):
    surf = cv2.SURF(400)
    kp,desc = surf.detectAndCompute(img ,None)
    img = cv2.drawKeypoints(img, kp, color=(255,0,0))
    return img

@staticmethod
def mserDetector(img):
    mser = cv2.FeatureDetector_create('MSER')
    kp = mser.detect(img)
    orb = cv2.DescriptorExtractor_create('ORB')
    kp, desc = orb.compute(img,kp)
    img = cv2.drawKeypoints(img, kp, color=(255,0,0))
    return img

@staticmethod
def generateSalientLAB(img):
    blur = cv2.GaussianBlur(img, (9,9), 2)
    l,a,b = cv2.split(cv2.cvtColor(blur, cv2.COLOR_LBGR2LAB))
    mean_l = np.full(img.shape[:2], np.mean(l))
    mean_a = np.full(img.shape[:2], np.mean(a))
    mean_b = np.full(img.shape[:2], np.mean(b))
    total = VUtil.euclid_dist([(l,mean_l),(a,mean_a),(b,mean_b)])
    total = np.uint8(total)
    return VUtil.toBGR(total, 'gray')

@staticmethod
def generateNewColor(img):
    b,g,r = cv2.split(img)
    c1 = VUtil.toBGR(np.uint8(np.arctan2(r,np.maximum(b,g))*255), 'gray')
    c2 = VUtil.toBGR(np.uint8(np.arctan2(g,np.maximum(r,b))*255), 'gray')
    c3 = VUtil.toBGR(np.uint8(np.arctan2(b,np.maximum(r,g))*255), 'gray')
    denominator = cv2.pow(r-g,2)+cv2.pow(r-b,2)+cv2.pow(g-b,2)
    l1 = VUtil.toBGR(cv2.pow(r-g,2)/denominator, 'gray')
    l2 = VUtil.toBGR(cv2.pow(r-b,2)/denominator, 'gray')
    l3 = VUtil.toBGR(cv2.pow(g-b,2)/denominator, 'gray')
    return np.vstack((np.hstack((c1,c2,c3)),np.hstack((l1,l2,l3))))

