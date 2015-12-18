#!/usr/bin/env python3

@staticmethod
def drawInfo(img, outData):
    y,x = img.shape[:2]
    center_y = y/2
    center_x = x/2
    outData['detected'] = True
    outData['dxy'][0] = (outData['centroid'][0] - center_x)/float(x)
    outData['dxy'][1] = (center_y - outData['centroid'][1])/float(y)
    VUtil.draw_circle(img, outData['centroid'],4,(0,244,255),-1)
    VUtil.draw_rect(img, (center_x - 5, center_y + 5), (center_x + 5, center_y - 5), (0,255,0), 2)


@staticmethod
def getBlack2(img):
    max_area = img.shape[0]*img.shape[1]
    white = np.zeros_like(img)
    cent = (-1,-1) 
    h,s,gray = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
    gray = VUtil.normIllumColor(gray, 0.5)
    max_pix = np.amax(gray)
    min_pix = np.amin(gray)
    diff = max_pix - min_pix
    thresh = min_pix + diff/3.5
    black = cv2.threshold(gray,thresh,255,cv2.THRESH_BINARY_INV)[1]
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    kern2 = cv2.getStructuringElement(cv2.MORPH_RECT, (1,1))
    mask = black 
    mask = cv2.erode(mask, kern, iterations=1)
    #mask = cv2.bitwise_not(mask)
    outImg = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    contours,hierr = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if len(contours) >= 1:
        hierr = hierr[0] 
        for component in zip(contours,hierr):
            currCnt = component[0]
            currHierr = component[1]
            parent = currHierr[3]
            currCnt = cv2.convexHull(currCnt)
            if 500 < VUtil.getRectArea(currCnt) < max_area/2 and VUtil.checkBins(currCnt,1.1):
                rect = cv2.minAreaRect(currCnt)
                cv2.drawContours(outImg, [currCnt], -1, (255,0,0), 4)
                cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))], -1, (0,0,255), 4)
                cv2.drawContours(white, [np.int0(cv2.cv.BoxPoints(rect))], -1, (255,255,255), -1)

    return white

@staticmethod
def findLane(img,info,blank):
    threshImg = VUtil.thresh_orange(img)
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    threshImg = cv2.erode(threshImg, kern, iterations=1)
    threshImg = cv2.dilate(threshImg, kern, iterations=2)
    outImg = cv2.cvtColor(threshImg, cv2.COLOR_GRAY2BGR)
    contours,_ = cv2.findContours(threshImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)
    if len(contours) >= 1:
        for cnt in contours:
            rect = cv2.minAreaRect(cnt)
            points = np.int32(cv2.cv.BoxPoints(rect))
            edge1 = points[1] - points[0]
            edge2 = points[2] - points[1]
            #Remove false positive by limiting area
            cnt = cv2.convexHull(cnt)
            if cv2.contourArea(cnt) > 700 and VUtil.checkRectangle(cnt):
                #Draw bounding rect
                cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))], -1, (255,0,0), 3)
                cv2.drawContours(blank, [np.int0(cv2.cv.BoxPoints(rect))], -1, ORANGE, 2)
                if cv2.norm(edge1) > cv2.norm(edge2):
                    rectAngle = math.degrees(math.atan2(edge1[1], edge1[0]))
                else:
                    rectAngle = math.degrees(math.atan2(edge2[1], edge2[0]))
                info['detected'] = True
                info['angle'] = 90 - abs(rectAngle) if rectAngle >= -90 else 90 - abs(rectAngle)
                info['centroid'] = VUtil.getCentroid(cnt)

                #Draw angle
                startpt = info['centroid']
                gradient = np.deg2rad(rectAngle)
                endpt = (int(startpt[0] + 200 * math.cos(gradient)),
                         int(startpt[1] + 200 * math.sin(gradient)))
                startpt = (int(startpt[0]), int(startpt[1]))
                cv2.line(outImg, startpt, endpt, (0,255,0),3)
                cv2.line(blank, startpt, endpt, GREEN,2)
                info['centroid'] = VUtil.getCentroid(cnt)
                VUtil.drawInfo(outImg,info)
                break
    return outImg

@staticmethod
def checkRectangle(cnt,ratio_limit=1.2):
    rect = cv2.minAreaRect(cnt)
    rect_area = rect[1][0]*rect[1][1]
    ratio = rect_area/(cv2.contourArea(cnt)+0.001)
    return ratio < ratio_limit

@staticmethod
def checkBins(cnt,ratio_limit=1.2):
    rect = cv2.minAreaRect(cnt)
    if rect[1][0] > rect[1][1]:
        asp_rat = rect[1][0]/rect[1][1]
    else:
        asp_rat = rect[1][1]/rect[1][0]
    rect_area = rect[1][0]*rect[1][1]
    ratio = rect_area/(cv2.contourArea(cnt)+0.001)
    return ratio < ratio_limit and asp_rat < 3

@staticmethod
def checkRectangle2(cnt):
    rect = cv2.minAreaRect(cnt)
    rect_area = rect[1][0]*rect[1][1]
    ratio = rect_area/(cv2.contourArea(cnt)+0.001)
    return ratio 

@staticmethod
def checkCircle(cnt):
    circle = cv2.minEnclosingCircle(cnt)
    circle_area = (circle[1]**2)*math.pi
    ratio = circle_area/(cv2.contourArea(cnt)+0.001)
    return ratio < 1.5

@staticmethod
def checkCircle2(cnt):
    circle = cv2.minEnclosingCircle(cnt)
    circle_area = (circle[1]**2)*math.pi
    ratio = circle_area/(cv2.contourArea(cnt)+0.001)
    return ratio 

@staticmethod
def checkThunder(cnt):
    rect = cv2.minAreaRect(cnt)
    if rect[1][0] > rect[1][1]:
        asp_rat = rect[1][0]/rect[1][1]
    else:
        asp_rat = rect[1][1]/rect[1][0]
    return -85 < rect[2] < -50 or asp_rat > 2
     
@staticmethod
def checkBanana(cnt):
    rect = cv2.minAreaRect(cnt)
    rect_area = rect[1][0]*rect[1][1]
    ratio = rect_area/(cv2.contourArea(cnt)+0.001)
    return 1.5 < ratio < 3.0

@staticmethod
def getTrain(threshImg,info,blank):
    info['detected'] = False
    data = []
    chosen_cnt = []
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    threshImg = cv2.dilate(threshImg, kern, iterations=1)
    outImg = cv2.cvtColor(threshImg, cv2.COLOR_GRAY2BGR)
    contours, _ = cv2.findContours(threshImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours.sort(key=VUtil.getRectArea, reverse=True)
    if len(contours) >= 1:
        info['heading'] = 0
        for x,cnt in enumerate(contours):
            if cv2.contourArea(cnt) > 500:
                data.append(VUtil.getCentroid(cnt))
                chosen_cnt.append(cnt)
                if x == 0:
                    info['centroid'] = VUtil.getCentroid(cnt)
                    info['area'] = VUtil.getRectArea(cnt)/float(outImg.shape[0]*outImg.shape[1])
                    VUtil.drawInfo(outImg,info)
                rect = cv2.minAreaRect(cnt)
                cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))], -1, (0,0,255), 1)
                cv2.drawContours(blank, [np.int0(cv2.cv.BoxPoints(rect))], -1, YELLOW, 2)

        if len(data) > 1:
            info['detected'] = True
            info['heading'] = data[0][0] - data[len(data)-1][0]
            info['heading'] = -1 if info['heading'] > 0 else 1
            deltaX = 20 if info['heading'] > 0  else -20
            info['centroid'] = VUtil.averageCentroids(data)
            cv2.circle(outImg, info['centroid'], 5, (0,0,255), -1)
            x = info['centroid'][0] 
            y = info['centroid'][1] + 15
    	    cv2.circle(outImg,(x,y),10,(255,0,255),2)
            VUtil.groupContours(chosen_cnt,outImg,info)
            info['centroid'] = (x,y)
            VUtil.drawInfo(outImg,info)

    return outImg

    
@staticmethod
def getDelorean(threshImg,info,blank):
    info['detected'] = False
    data = []
    chosen_cnt = []
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    threshImg = cv2.dilate(threshImg, kern, iterations=1)
    outImg = cv2.cvtColor(threshImg, cv2.COLOR_GRAY2BGR)
    contours, _ = cv2.findContours(threshImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours.sort(key=VUtil.getRectArea, reverse=True)
    if len(contours) >= 1:
        info['heading'] = 0
        for x,cnt in enumerate(contours):
            if cv2.contourArea(cnt) > 500:
                data.append(cnt)
                if VUtil.getAspRatio(cnt) < 2.5 and x <= 1:
                    info['centroid'] = VUtil.getCentroid(cnt)
                    #info['area'] = VUtil.getRectArea(cnt)
                    info['area'] = VUtil.getRectArea(cnt)/float(outImg.shape[0]*outImg.shape[1])
                    #VUtil.getDOA(cnt,outImg,info)
                    VUtil.drawInfo(outImg,info)
                rect = cv2.minAreaRect(cnt)
                #cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))], -1, (0,0,255), 1)
                cv2.drawContours(blank, [np.int0(cv2.cv.BoxPoints(rect))], -1, ORANGE, 2)

        if len(data) > 1:
            info['detected'] = True
            chosen_cnt = [VUtil.getCentroid(x) for x in data]
            combine = zip(data,chosen_cnt)
            if len(data) > 2:
                combine.sort(key=lambda x:cv2.contourArea(x[0]), reverse=True)
                for i in combine:
                    cnt = i[0]
                    if VUtil.getAspRatio(cnt) < 2.5:
                        rect = cv2.minAreaRect(cnt)
                        cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))], -1, PURPLE, 2)
                        biggest = VUtil.getCentroid(cnt)
                        break
                combine.sort(key=lambda x:cv2.norm(biggest,x[1]))
                data = [x[0] for x in combine[:2]]
                chosen_cnt = [x[1] for x in combine[:2]]
            info['heading'] = chosen_cnt[0][0] - chosen_cnt[1][0]
            info['heading'] = -1 if info['heading'] > 0 else 1
            deltaX = 40 if info['heading'] < 0  else -40
            info['centroid'] = VUtil.averageCentroids(chosen_cnt)
            cv2.circle(outImg, info['centroid'], 5, (0,0,255), -1)
            x = info['centroid'][0] 
            y = info['centroid'][1] + 10
    	    cv2.circle(outImg,(x,y),10,(255,0,255),2)
            VUtil.groupContours(data,outImg,info)
            info['centroid'] = (x,y)
            VUtil.drawInfo(outImg,info)

    return outImg


@staticmethod
def getBinsShape(mask,info,blank):
    info['pattern'] = [-1,-1,-1,-1]
    data = []
    chosen_cnt = []
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    outImg = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    contours, hierr = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if len(contours) >= 1:
        hierr = hierr[0]
        for cnt,hirr in zip(contours,hierr):
            parent = hirr[3]
            child = hirr[2]
            if parent != -1 and cv2.contourArea(cnt) > 100 and VUtil.checkRectangle(contours[parent]):
                #data.append(VUtil.getCentroid(cnt))
                chosen_cnt.append(cnt)
                rect = cv2.minAreaRect(cnt)
                #cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))], -1, (0,0,255), 2)
                info['centroid'] = VUtil.getCentroid(cnt)
                VUtil.drawInfo(outImg,info)
                #VUtil.getDOA(cnt,outImg,info)

        if len(data) > 1:
            info['centroid'] = VUtil.averageCentroids(data)
            cv2.circle(outImg, info['centroid'], 5, (0,0,255), -1)
            cv2.circle(outImg, info['centroid'], 10, (0,0,255), -1)
            VUtil.drawInfo(outImg,info)
        VUtil.classify_cnt(outImg, info, blank, chosen_cnt)
    return outImg

@staticmethod
def getBinsBlack(threshImg):
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    y,x = threshImg.shape[:2]
    black = np.zeros((y,x,3), dtype=np.uint8)
    outImg = cv2.cvtColor(threshImg, cv2.COLOR_GRAY2BGR)
    contours, hierr = cv2.findContours(threshImg, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)
    if len(contours) >= 1:
        hierr = hierr[0]
        for cnt,hirr in zip(contours,hierr):
            rect = cv2.minAreaRect(cnt)
            parent = hirr[3]
            if VUtil.checkRectangle(cnt) and parent != -1 and cv2.contourArea(cnt) > 400:
                cv2.drawContours(black, [np.int0(cv2.cv.BoxPoints(rect))], -1, (255,255,255), -1)
                #cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))], -1, (255,0,255), 2)
    return black

@staticmethod
def getBins(threshImg,info,blank):
    info['pattern'] = [-1,-1,-1,-1]
    data = []
    chosen_cnt = []
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    threshImg = cv2.dilate(threshImg, kern, iterations=1)
    outImg = cv2.cvtColor(threshImg, cv2.COLOR_GRAY2BGR)
    contours, _ = cv2.findContours(threshImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours.sort(key=VUtil.getRectArea, reverse=True)
    if len(contours) >= 1:
        info['heading'] = 0
        for x,cnt in enumerate(contours):
            cnt = cv2.convexHull(cnt)
            cnt = cv2.approxPolyDP(cnt, 0.001*cv2.arcLength(cnt,True),True)
            rect = cv2.minAreaRect(cnt)
            area = rect[1][0]*rect[1][1]
            if cv2.contourArea(cnt) > 300 and x < 4:
                data.append(VUtil.getCentroid(cnt))
                chosen_cnt.append(cnt)
                if x == 0:
                    info['centroid'] = VUtil.getCentroid(cnt)
                    info['area'] = cv2.minAreaRect(cnt)[2]/float(threshImg.shape[0]*threshImg.shape[1])
                    VUtil.drawInfo(outImg,info)
                    VUtil.getDOA(cnt,outImg,info)
                rect = cv2.minAreaRect(cnt)
                cv2.drawContours(outImg, [cnt], -1, (255,0,255), 2)
                cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))], -1, (0,0,255), 2)

        if len(data) > 1:
            info['heading'] = data[0][0] - data[len(data)-1][0]
            deltaX = 40 if info['heading'] < 0  else -40
            info['centroid'] = VUtil.averageCentroids(data)
            cv2.circle(outImg, info['centroid'], 5, (0,0,255), -1)
            cv2.circle(outImg, info['centroid'], 10, (0,0,255), -1)
            VUtil.groupContours(chosen_cnt,outImg,info)
            VUtil.drawInfo(outImg,info)

        VUtil.classify_cnt(outImg, info, blank, chosen_cnt)
    return outImg

@staticmethod
def getOverallBins(threshImg,info,blank):
    data = []
    chosen_cnt = []
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    threshImg = cv2.dilate(threshImg, kern, iterations=1)
    outImg = cv2.cvtColor(threshImg, cv2.COLOR_GRAY2BGR)
    contours, _ = cv2.findContours(threshImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours.sort(key=cv2.contourArea, reverse=True)
    if len(contours) >= 1:
        info['heading'] = 0
        for x,cnt in enumerate(contours):
            if cv2.contourArea(cnt) > 300 and x < 4:
                data.append(VUtil.getCentroid(cnt))
                chosen_cnt.append(cnt)
                if x == 0:
                    info['centroid'] = VUtil.getCentroid(cnt)
                    info['area'] = cv2.minAreaRect(cnt)[2]/float(threshImg.shape[0]*threshImg.shape[1])
                    VUtil.drawInfo(outImg,info)
                    VUtil.getDOA(cnt,outImg,info)
                rect = cv2.minAreaRect(cnt)
                cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))], -1, (0,0,255), 1)
                cv2.drawContours(blank, [np.int0(cv2.cv.BoxPoints(rect))], -1, YELLOW, 2)

        if len(data) > 1:
            info['centroid'] = VUtil.averageCentroids(data)
            cv2.circle(outImg, info['centroid'], 5, (0,0,255), -1)
            cv2.circle(outImg, info['centroid'], 10, (0,0,255), -1)
            VUtil.groupContoursAlign(chosen_cnt,outImg,info,blank)
            VUtil.drawInfo(outImg,info)

    return outImg

@staticmethod
def getCover(threshImg,info,blank):
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    threshImg = cv2.erode(threshImg, kern, iterations=1)
    threshImg = cv2.morphologyEx(threshImg, cv2.MORPH_CLOSE,kern, iterations=1)
    outImg = cv2.cvtColor(threshImg, cv2.COLOR_GRAY2BGR)
    contours, _ = cv2.findContours(threshImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours.sort(key=VUtil.getRectArea, reverse=True)
    if len(contours) >= 1:
        for x,cnt in enumerate(contours):
            if cv2.contourArea(cnt) > 300:
                rect = cv2.minAreaRect(cnt)
                x,y = VUtil.getCentroid(cnt)
                info['centroid'] = (x, y)
                info['area'] = rect[1][1]*rect[1][0]/float(threshImg.shape[0]*threshImg.shape[1])
                VUtil.drawInfo(outImg,info)
                VUtil.getDOA(cnt,outImg,info)
                rect = cv2.minAreaRect(cnt)
                cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))], -1, (0,0,255), 1)
                cv2.drawContours(blank, [np.int0(cv2.cv.BoxPoints(rect))], -1, ORANGE, 2)
                break

    return outImg

@staticmethod
def classify_cnt(img, info, blank, chosen_cnt):
    y,x = img.shape[:2]
    center_y = y/2
    center_x = x/2
    info['pattern'] = [-1,-1,-1,-1]
    '''
    0  = mr.fusion, 1 = banana, 2 = cola, 3 = thunder
    '''
    chosen_cnt.sort(key=VUtil.getRectArea,reverse=True)

    for cnt in chosen_cnt:

        cent = VUtil.getCentroid(cnt)
    	deltaX = (cent[0] - center_x)/float(x)
    	deltaY = (center_y - cent[1])/float(y)
        rect = cv2.minAreaRect(cnt)
        area = (rect[1][1]*rect[1][0])/float(y*x)

        if VUtil.checkCircle(cnt) and info['pattern'][0] == -1:
            info['pattern'][0] = (deltaX,deltaY,area)
            cv2.circle(img, cent, 5, PURPLE, -1)
            cv2.circle(blank, cent, 5, PURPLE, -1)
            cv2.drawContours(blank, [cnt] , -1, PURPLE, 2)

        if VUtil.checkThunder(cnt) and info['pattern'][3] == -1:
            info['pattern'][3] = (deltaX,deltaY,area)
            cv2.circle(img, cent, 5, BLUE, -1)
            cv2.circle(blank, cent, 5, BLUE, -1)
            cv2.drawContours(blank, [cnt] , -1, BLUE, 2)

        if VUtil.checkRectangle(cnt) and info['pattern'][2] == -1:
            info['pattern'][2] = (deltaX,deltaY,area)
            cv2.circle(img, cent, 5, RED, -1)
            cv2.circle(blank, cent, 5, RED, -1)
            cv2.drawContours(blank, [cnt] , -1, RED, 2)

        if VUtil.checkBanana(cnt) and info['pattern'][1] == -1:
            info['pattern'][1] = (deltaX,deltaY,area)
            cv2.circle(img, cent, 5, GREEN, -1)
            cv2.circle(blank, cent, 5, GREEN, -1)
            cv2.drawContours(blank, [cnt] , -1, GREEN, 2)

@staticmethod
def getDeltaArea(cnt,x,y):
    center_x = x/2
    center_y = y/2
    cent = VUtil.getCentroid(cnt)
    deltaX = (cent[0] - center_x)/float(x)
    deltaY = (center_y - cent[1])/float(y)
    rect = cv2.minAreaRect(cnt)
    area = (rect[1][1]*rect[1][0])/float(y*x)
    return (deltaX,deltaY,area)

@staticmethod
def findTrain(img,info,blank):
    mask = VUtil.thresh_yellow(img)
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3,3)))
    return VUtil.getTrain(mask, info,blank)

@staticmethod
def findDelorean(img,info,blank):
    mask = VUtil.thresh_orange(img)
    return VUtil.getDelorean(mask,info,blank)

@staticmethod
def findOverallBins(img, info, blank,mode=1):
    white = VUtil.getBlack2(img)
    mask = cv2.cvtColor(white, cv2.COLOR_BGR2GRAY)
    if mode != 1:
        mask = mask & VUtil.thresh_yellow(img)
    return VUtil.getOverallBins(mask,info,blank)

@staticmethod
def findBins3(img, info, blank,mode=1):
    h,s,v = cv2.split(VUtil.toHSV(img))
    v = cv2.GaussianBlur(v, (3,3),0)
    black = VUtil.getBinsBlack(VUtil.adaptThresh(v,11,10))
    yellow = VUtil.getPatternHue(VUtil.iace(img))
    orange = cv2.bitwise_not(VUtil.thresh_orange2(img))
    mask = yellow & cv2.cvtColor(black, cv2.COLOR_BGR2GRAY) & orange
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)), iterations=2)
    return VUtil.getBins(mask,info,blank)

@staticmethod
def findBins2(img, info, blank,mode=1):
    white = VUtil.getBlack2(img)
    mask = cv2.cvtColor(white, cv2.COLOR_BGR2GRAY)
    mask = mask & VUtil.getPatternHue(cv2.GaussianBlur(VUtil.iace(img),(5,5),0))
    orange = VUtil.thresh_orange2(img)
    yellow = VUtil.thresh_yellow(img,1,110)
    mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)),iterations=2) & cv2.bitwise_not(orange) & yellow
    return VUtil.getBins(mask,info,blank)

@staticmethod
def findBins(img,info,blank,mode=1):
    img = VUtil.finlaynorm(VUtil.iace(img))
    black = cv2.cvtColor(VUtil.getBinsBlack(cv2.bitwise_not(VUtil.filterBlack(img))),cv2.COLOR_BGR2GRAY)
    black = cv2.cvtColor(VUtil.getBinsBlack(VUtil.filterBlack(img)),cv2.COLOR_BGR2GRAY)
    return VUtil.toBGR(black, 'gray')
    mask = VUtil.getPatternHue(img)
    #mask = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT,(3,3)))
    return VUtil.getBinsShape(mask,info,blank)

@staticmethod
def findCover(img,info,blank):
    mask = VUtil.thresh_orange2(img)
    return VUtil.getCover(mask,info,blank)

@staticmethod
def findRail(img,info,blank):
    h,s,v = cv2.split(VUtil.toHSV(img))
    return VUtil.detectRail(v,info,blank)

@staticmethod
def findTracks(img):
    h,s,v = cv2.split(cv2.cvtColor(img, cv2.COLOR_BGR2HSV))
    return VUtil.cannyChild(v)

@staticmethod
def identifyObject(img,info,blank):
    threshImg = VUtil.thresh_yellow(img)
    info['object'] = 0
    data = []
    chosen_cnt = []
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    threshImg = cv2.dilate(threshImg, kern, iterations=1)
    outImg = cv2.cvtColor(threshImg, cv2.COLOR_GRAY2BGR)
    contours, _ = cv2.findContours(threshImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours.sort(key=VUtil.getRectArea, reverse=True)
    if len(contours) >= 1:
        for x,cnt in enumerate(contours):
            if cv2.contourArea(cnt) > 600:
                data.append(VUtil.getCentroid(cnt))
                chosen_cnt.append(cnt)
                rect = cv2.minAreaRect(cnt)
                cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))], -1, (0,0,255), 1)

        if len(data) > 1:
            info['detected'] = True
            hull = cv2.convexHull(np.vstack(chosen_cnt))
            rect = cv2.minAreaRect(hull)
            ellipse = cv2.fitEllipse(hull)
            cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))],-1,(255,0,255),3)
            cv2.drawContours(blank, [np.int0(cv2.cv.BoxPoints(rect))],-1,PURPLE,2)
            aspect_ratio = ellipse[1][1]/ellipse[1][0]
            print("Aspect ratio: " + str(aspect_ratio))
            if aspect_ratio > 1.7:
                info['object'] = 1
            elif aspect_ratio < 1.5:
                info['object'] = -1

    return outImg

@staticmethod
def detectGeneral(img,info,blank):
    threshImg = VUtil.thresh_yellow2(img)
    threshImg = cv2.erode(threshImg, cv2.getStructuringElement(cv2.MORPH_RECT, (3,3)))
    data = []
    chosen_cnt = []
    kern = cv2.getStructuringElement(cv2.MORPH_RECT, (3,3))
    threshImg = cv2.dilate(threshImg, kern, iterations=1)
    outImg = cv2.cvtColor(threshImg, cv2.COLOR_GRAY2BGR)
    contours, _ = cv2.findContours(threshImg, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours.sort(key=VUtil.getRectArea, reverse=True)
    if len(contours) >= 1:
        for x,cnt in enumerate(contours):
            if cv2.contourArea(cnt) > 600 and VUtil.checkRectangle(cnt):
                info['detected'] = True
                data.append(VUtil.getCentroid(cnt))
                chosen_cnt.append(cnt)
                rect = cv2.minAreaRect(cnt)
                cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))], -1, (0,0,255), 3)
                info['object'] = 1
                break


        '''
        if len(data) > 1:
            hull = cv2.convexHull(np.vstack(chosen_cnt))
            rect = cv2.minAreaRect(hull)
            ellipse = cv2.fitEllipse(hull)
            cv2.drawContours(outImg, [np.int0(cv2.cv.BoxPoints(rect))],-1,(255,0,255),3)
            cv2.drawContours(blank, [np.int0(cv2.cv.BoxPoints(rect))],-1,PURPLE,2)
            aspect_ratio = ellipse[1][1]/ellipse[1][0]
            print("Aspect ratio: " + str(aspect_ratio))
            if aspect_ratio > 1.7:
                info['object'] = 1
            elif aspect_ratio < 1.5:
                info['object'] = -1
                '''

    return outImg

@staticmethod
def binSequence(img,info,blank):
    overall = VUtil.findOverallBins(img,info,blank)
    cover = VUtil.findCover(img,info,blank)
    bins = VUtil.findBins(img,info,blank)
    return np.hstack((overall,cover,bins))

@staticmethod
def homeSequence(img,info,blank):
    train = VUtil.findTrain(img,info,blank)
    delorean = VUtil.findDelorean(img,info,blank)
    general = VUtil.detectGeneral(img,info,blank)
    return np.hstack((general,train,delorean))
