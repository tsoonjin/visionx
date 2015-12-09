#!/usr/bin/env python3

@staticmethod 
def getAspRatio(cnt):
    rect = cv2.minAreaRect(cnt)
    w,l = rect[1]
    if l > w:
        ratio = l/w
    else:
        ratio= w/l
    print(ratio)
    return ratio

@staticmethod
def box2D(rect):
    box = cv2.cv.BoxPoints(rect)
    return np.int0(box)

@staticmethod
def groupContours(chosen_cnt, outImg, info):
    hull = cv2.convexHull(np.vstack(chosen_cnt))
    info['area'] = VUtil.getRectArea(hull)/float(outImg.shape[0]*outImg.shape[1])
    VUtil.getDOA(hull,outImg,info)

@staticmethod
def groupContoursPickup(chosen_cnt, outImg, info):
    hull = cv2.convexHull(np.vstack(chosen_cnt))
    info['area'] = VUtil.getRectArea(hull)
    print(info['area'])
    VUtil.getDOA(hull,outImg,info)

@staticmethod
def groupContoursAlign(chosen_cnt, outImg, info,blank):
    hull = cv2.convexHull(np.vstack(chosen_cnt))
    info['area'] = VUtil.getRectArea(hull)/float(outImg.shape[0]*outImg.shape[1])
    VUtil.getRailDOA(hull,outImg,info,blank)

@staticmethod
def getDOA(cnt,outImg,info):
    rect = cv2.minAreaRect(cnt)
    points = np.int32(cv2.cv.BoxPoints(rect))
    edge1 = points[1] - points[0]
    edge2 = points[2] - points[1]
    if cv2.norm(edge1) > cv2.norm(edge2):
        rectAngle = math.degrees(math.atan2(edge1[1], edge1[0]))
    else:
        rectAngle = math.degrees(math.atan2(edge2[1], edge2[0]))
    startpt = info['centroid']
    gradient = np.deg2rad(rectAngle)
    endpt = (int(startpt[0] + 200 * math.cos(gradient)),
             int(startpt[1] + 200 * math.sin(gradient)))
    startpt = (int(startpt[0]), int(startpt[1]))
    cv2.line(outImg, startpt, endpt, (0,255,0),2)
    cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))],-1,(255,0,0),2)
    info['angle'] = 90 - abs(rectAngle) if rectAngle >= -90 else  90 - abs(rectAngle)
    info['angle'] = (info['angle']+90)%360
    
@staticmethod
def getRailDOA(cnt,outImg,info,blank):
    rect = cv2.minAreaRect(cnt)
    points = np.int32(cv2.cv.BoxPoints(rect))
    edge1 = points[1] - points[0]
    edge2 = points[2] - points[1]
    if cv2.norm(edge1) > cv2.norm(edge2):
        rectAngle = math.degrees(math.atan2(edge1[1], edge1[0]))
    else:
        rectAngle = math.degrees(math.atan2(edge2[1], edge2[0]))
    startpt = info['centroid']
    gradient = np.deg2rad(rectAngle)
    endpt = (int(startpt[0] + 200 * math.cos(gradient)),
             int(startpt[1] + 200 * math.sin(gradient)))
    startpt = (int(startpt[0]), int(startpt[1]))
    cv2.line(outImg, startpt, endpt, (0,255,0),2)
    cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))],-1,(255,0,0),2)
    cv2.drawContours(blank, [np.int0(cv2.cv.BoxPoints(rect))],-1,BLUE,2)
    info['angle'] = 90 - abs(rectAngle) if rectAngle >= -90 else  90 - abs(rectAngle)
    #info['angle'] = (info['angle']-90)%360

@staticmethod
def averageCentroids(centroids):
    x = int(sum(c[0] for c in centroids)/float(len(centroids)))
    y = int(sum(c[1] for c in centroids)/float(len(centroids)))
    return (x,y)

@staticmethod
def getRectArea(cnt):
    rect = cv2.minAreaRect(cnt)
    return int(rect[1][0]*rect[1][1])

@staticmethod
def getCorner(box):
    x = [i[0] for i in box]
    y = [i[1] for i in box]
    top_left = (min(x), max(y))
    top_right = (max(x), max(y))
    bot_right = (max(x), min(y))
    bot_left = (min(x), min(y))
    return [top_left, top_right, bot_left, bot_right]

@staticmethod
def approxCnt(cnt,offset=0.05):
    """lower offset yields better approximation"""
    epsilon = offset*cv2.arcLength(cnt, True)
    approx = cv2.approxPolyDP(cnt, epsilon, True)
    return approx

@staticmethod
def getCentroid(cnt):
    mom = cv2.moments(cnt)
    centroid_x = int((mom['m10']+0.0001)/(mom['m00']+0.0001))
    centroid_y = int((mom['m01']+0.0001)/(mom['m00']+0.0001))
    return (centroid_x, centroid_y)

@staticmethod
def getCovexity(cnts):
	P = cv2.arcLength(cnts, True)
	P_convex = cv2.arcLength(cv2.convexHull(cnts),True)
	return P_convex/P

@staticmethod
def getHu(cnts):
	return cv2.HuMoments(cv2.moments(cnts))

@staticmethod
def getCompactness(cnts):
	P_circle = ((cv2.contourArea(cnts)*math.pi)**0.5)*2
	P = cv2.arcLength(cnts, True)
	return P_circle/P

@staticmethod 
def detectRailBlack(img, info,blank):
    cent = (-1,-1)
    mask = VUtil.filterBlack(img)
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    outImg = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    contours, hierr = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours.sort(key=cv2.contourArea, reverse=True) 

    if len(contours) >= 1:
        for currCnt in contours:
            rect = cv2.minAreaRect(currCnt)
            if cv2.contourArea(currCnt) > 5000:
                info['centroid'] =   VUtil.getCentroid(currCnt)
                VUtil.drawInfo(outImg,info)
                VUtil.getRailDOA(currCnt,outImg,info,blank)
                break
    return outImg

@staticmethod 
def detectRail(gray, info,blank):
    chosen_cnt = []
    chosen_cntx = []
    cent = (-1,-1)
    gray = cv2.GaussianBlur(gray, (3,3),0)
    #gray = cv2.GaussianBlur(gray, (9,9),2)
    area = gray.shape[0]*gray.shape[1]
    min = np.amin(gray)
    max = np.amax(gray)
    thresh = min + (max-min)/1.5
    mask = np.uint8(cv2.Canny(gray, thresh/2, thresh))
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    mask = cv2.dilate(mask, kern, iterations=1)
    #mask = cv2.erode(mask, kern, iterations=1)
    outImg = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    contours, hierr = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contours.sort(key=cv2.contourArea, reverse=True) 

    info['detected'] = False
    if len(contours) >= 1:
        for currCnt in contours:
            rect = cv2.minAreaRect(currCnt)

            if 300 < cv2.contourArea(currCnt) < 3000 and VUtil.checkRectangle(currCnt):
                chosen_cnt.append(VUtil.getCentroid(currCnt))
                chosen_cntx.append(currCnt)
                info['centroid'] =   VUtil.getCentroid(currCnt)
                VUtil.drawInfo(outImg,info)
                #VUtil.getRailDOA(currCnt,outImg,info,blank)
        if len(chosen_cnt) > 1:
            info['detected'] = True
            info['centroid'] = VUtil.averageCentroids(chosen_cnt)
            VUtil.groupContoursAlign(chosen_cntx,outImg,info,blank)
            VUtil.drawInfo(outImg,info)

        print(info['detected'])
    return outImg

@staticmethod 
def detectSmallSquare(gray, info,blank,sm=50):
    chosen_cnt = []
    chosen_cntx = []
    cent = (-1,-1)
    gray = cv2.GaussianBlur(gray, (3,3),0)
    #gray = cv2.GaussianBlur(gray, (9,9),2)
    area = gray.shape[0]*gray.shape[1]
    min = np.amin(gray)
    max = np.amax(gray)
    thresh = min + (max-min)/1.5
    mask = np.uint8(cv2.Canny(gray, thresh/2, thresh))
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    mask = cv2.dilate(mask, kern, iterations=1)
    #mask = cv2.erode(mask, kern, iterations=1)
    outImg = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    contours, hierr = cv2.findContours(mask, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)
    contours.sort(key=cv2.contourArea, reverse=True) 

    if len(contours) >= 1:
        for currCnt in contours:
            rect = cv2.minAreaRect(currCnt)
            ellipse = cv2.fitEllipse(currCnt)
            #if cv2.contourArea(currCnt) > 5000 and (ellipse[1][1]/ellipse[1][0]) >= 3:
            if cv2.contourArea(currCnt) > 1000 and VUtil.checkRectangle(currCnt):
                info['detected'] = True
                cent = VUtil.getCentroid(currCnt)
                cent = (cent[0],cent[1]-70)
                chosen_cnt.append(cent)
                chosen_cntx.append(currCnt)
                info['centroid'] =   cent
                VUtil.drawInfo(outImg,info)
                VUtil.getRailDOA(currCnt,outImg,info,blank)

        if len(chosen_cnt) > 1:
            info['detected'] = True
            info['centroid'] = VUtil.averageCentroids(chosen_cnt)
            VUtil.groupContoursAlign(chosen_cntx,outImg,info,blank)
            chosen_cnt.sort(key=lambda x:x[0], reverse=True)
            chosen_cntx.sort(key=cv2.contourArea, reverse=True)
            chosen_cntx.sort(key=lambda x:VUtil.getCentroid(x)[0],reverse=True)
            rect_r = cv2.minAreaRect(chosen_cntx[0])
            rect_l = cv2.minAreaRect(chosen_cntx[-1])
            cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect_l))], -1, PURPLE,3)
            cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect_r))], -1, YELLOW,3)
            cv2.drawContours(blank, [np.int0(cv2.cv.BoxPoints(rect_l))], -1, PURPLE,2)
            cv2.drawContours(blank, [np.int0(cv2.cv.BoxPoints(rect_r))], -1, YELLOW,2)
            if sm < 0:
                info['centroid'] = chosen_cnt[-1]
            else:
                info['centroid'] = chosen_cnt[0]
            VUtil.drawInfo(outImg,info)
    return outImg

@staticmethod 
def detectEdge(gray):
    blur1 = cv2.GaussianBlur(gray,(5,5),2)
    blur2 = cv2.GaussianBlur(gray,(5,5),5)
    laplacian = VUtil.toBGR(blur1-blur2,'gray')
    min = np.min(gray)
    max = np.max(gray)
    thresh = min + (max-min)
    canny = VUtil.toBGR(np.uint8(cv2.Canny(gray, thresh/2,thresh)),'gray')
    return np.hstack((laplacian,canny))


@staticmethod
def getRailBox(img, info,blank,sm=50):
    chosen_cnt = []
    chosen_cntx = []
    h,s,v = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
    v = cv2.GaussianBlur(v, (5,5),0)
    v = cv2.GaussianBlur(v, (9,9),0)
    threshImg = cv2.adaptiveThreshold(v,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,21,5)
    threshImg = cv2.erode(threshImg, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)), iterations=1)
    outImg = VUtil.toBGR(threshImg, 'gray')
    contours, hierr = cv2.findContours(threshImg, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) >= 1:
        hierr = hierr[0]
        for component in zip(contours,hierr):
            currCnt = component[0]
            parent = component[1][3]
            rect = cv2.minAreaRect(currCnt)
            currCnt = cv2.convexHull(currCnt)
            if cv2.contourArea(currCnt) > 2000 and VUtil.checkRectangle(currCnt):
                chosen_cnt.append(VUtil.getCentroid(currCnt))
                chosen_cntx.append(currCnt)
                cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))], -1, (0,0,255),3)
                cv2.drawContours(blank, [np.int0(cv2.cv.BoxPoints(rect))], -1, RED,2)

        if chosen_cnt:
            chosen_cnt.sort(reverse=True)
            chosen_cntx.sort(key=cv2.contourArea, reverse=True)
            chosen_cntx.sort(key=lambda x:VUtil.getCentroid(x)[0],reverse=True)
            rect_l = cv2.minAreaRect(chosen_cntx[0])
            rect_r = cv2.minAreaRect(chosen_cntx[-1])
            cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect_l))], -1, BLUE,3)
            cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect_r))], -1, BLUE,3)
            cv2.drawContours(blank, [np.int0(cv2.cv.BoxPoints(rect_l))], -1, BLUE,2)
            cv2.drawContours(blank, [np.int0(cv2.cv.BoxPoints(rect_r))], -1, BLUE,2)
            cent = VUtil.averageCentroids(chosen_cnt)
            info['centroid'] = (cent[0],cent[1])
            VUtil.drawInfo(outImg,info)
            #VUtil.getDOA(chosen_cntx[0],outImg,info)
    return outImg

@staticmethod
def getRailBox2(img, info, blank, sm=50):
    chosen_cnt = []
    chosen_cntx = []
    h,s,v = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
    threshImg = cv2.adaptiveThreshold(v,255,cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY,11,5)
    threshImg = cv2.erode(threshImg, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)), iterations=1)
    outImg = VUtil.toBGR(threshImg, 'gray')
    contours, hierr = cv2.findContours(threshImg, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) >= 1:
        hierr = hierr[0]
        for component in zip(contours,hierr):
            currCnt = component[0]
            parent = component[1][3]
            rect = cv2.minAreaRect(currCnt)
            if cv2.contourArea(currCnt) > 1000 and VUtil.checkRectangle(currCnt):
                chosen_cnt.append(VUtil.getCentroid(currCnt))
                chosen_cntx.append(currCnt)
                cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))], -1, (0,0,255),3)
                cv2.drawContours(blank, [np.int0(cv2.cv.BoxPoints(rect))], -1, RED, 2)
        if chosen_cnt:
            chosen_cnt.sort(key=lambda x:x[0],reverse=True)
            chosen_cntx.sort(key=cv2.contourArea, reverse=True)
            if sm > 0:
                cent = chosen_cnt[1]
            else:
                cent = chosen_cnt[-2]
            info['centroid'] = (cent[0], cent[1]-50)
            VUtil.drawInfo(outImg,info)
    return outImg

@staticmethod
def getRail(img):
    info = dict()
    chosen_cnt = []
    h,s,v = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
    threshImg = cv2.adaptiveThreshold(v,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY,11,5)
    threshImg = cv2.erode(threshImg, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)), iterations=1)
    outImg = VUtil.toBGR(threshImg, 'gray')
    contours, hierr = cv2.findContours(threshImg, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) >= 1:
        hierr = hierr[0]
        for component in zip(contours,hierr):
            currCnt = component[0]
            parent = component[1][3]
            rect = cv2.minAreaRect(currCnt)
            if parent != -1 and cv2.contourArea(currCnt) > 1500:
                chosen_cnt.append(VUtil.getCentroid(currCnt))
                cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))], -1, (0,0,255),3)
        if chosen_cnt:
            #cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))], -1, (0,0,255),3)
            cent = VUtil.averageCentroids(chosen_cnt)
            info['centroid'] = (cent[0],cent[1])
            VUtil.drawInfo(outImg,info)
    return outImg,info

@staticmethod 
def distanceTransform(gray):
    dt = cv2.distanceTransform(gray, cv2.cv.CV_DIST_L2,5)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

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

