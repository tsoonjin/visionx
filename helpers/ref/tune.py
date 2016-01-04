#!/usr/bin/env python

import rospy
import sys
import subprocess
import signal
import os
import numpy as np
import cv2
from sensor_msgs.msg import Image,CompressedImage
from PyQt4 import QtGui,QtCore
from common import VUtil,Info

class Unit:

    def __init__(self,name,parent):
        self.name = name
        self.pub = rospy.Publisher('/%s/compressed'%name, CompressedImage)
        self.sub = None
        self.detect = getattr(VUtil, name) 
        self.parent = parent
        self.kern = (3,3)
        self.thresh = 100
        self.canPublish = False
        self.box = getCheckBox(name)
        self.box.stateChanged.connect(self.box_cb)
        

    def cam_cb(self,rosimg):
        cvimg = VUtil.rosimg2cv(rosimg)
        y,x = cvimg.shape[:2]
        cvimg = cv2.resize(cvimg, (x/2,y/2))
        if self.canPublish:
            out = self.detect(cvimg)
            self.pub.publish(VUtil.writeCompressed(out))

    def box_cb(self, state):
        if state == QtCore.Qt.Checked:
            topic = str(self.parent.currTopic)
            self.sub = rospy.Subscriber(topic,Image,self.cam_cb)
            self.canPublish = True
            self.parent.analyseTopic = '/%s/compressed'%self.name
            self.parent.active[self.name] = self
            self.ps = subprocess.Popen(['rosrun','image_view','image_view','image:='+'/%s'%self.name,'compressed'])
        else:
            self.canPublish = False
            self.parent.active.pop(self.name)
            self.ps.terminate()
            self.sub.unregister()

class Sub:

    def __init__(self,name,parent):
        self.name = name
        self.pub = rospy.Publisher('/%s/compressed'%name, CompressedImage)
        self.blank_pub = rospy.Publisher('/%s_blank/compressed'%name, CompressedImage)
        self.info = Info()
        self.sub = None
        self.detect = getattr(VUtil, name) 
        self.parent = parent
        self.kern = (3,3)
        self.thresh = 100
        self.canPublish = False
        self.box = getCheckBox(name)
        self.box.stateChanged.connect(self.box_cb)
        

    def cam_cb(self,rosimg):
        cvimg = VUtil.rosimg2cv(rosimg)
        y,x = cvimg.shape[:2]
        cvimg = cv2.resize(cvimg, (x/2,y/2))
        if self.canPublish:
            blank = np.zeros_like(cvimg)
            out = self.detect(cvimg,self.info.data,blank)
            self.info.draw(blank)
            self.pub.publish(VUtil.writeCompressed(out))
            self.blank_pub.publish(VUtil.writeCompressed(blank))

    def box_cb(self, state):
        if state == QtCore.Qt.Checked:
            topic = str(self.parent.currTopic)
            self.sub = rospy.Subscriber(topic,Image,self.cam_cb)
            self.canPublish = True
            self.parent.analyseTopic = '/%s/compressed'%self.name
            self.ps = subprocess.Popen(['rosrun','image_view','image_view','image:='+'/%s_blank'%self.name,'compressed'])
        else:
            self.canPublish = False
            self.sub.unregister()
            self.ps.terminate()

class Analyze:

    def __init__(self,name,parent):
        self.name = name
        self.pub = rospy.Publisher('/%s/compressed'%name, CompressedImage)
        self.sub = None
        self.detect = getattr(VUtil, name) 
        self.parent = parent
        self.kern = (3,3)
        self.thresh = 100
        self.canPublish = False
        self.box = getCheckBox(name)
        self.box.stateChanged.connect(self.box_cb)
        

    def cam_cb(self,rosimg):
        if 'compressed' in self.parent.analyseTopic:
            cvimg = VUtil.readCompressed(rosimg)
        else:
            cvimg = VUtil.rosimg2cv(rosimg)
        y,x = cvimg.shape[:2]
        cvimg = cv2.resize(cvimg, (x/2,y/2))
        if self.canPublish:
            out = self.detect(cvimg)
            self.pub.publish(VUtil.writeCompressed(out))

    def box_cb(self, state):
        if state == QtCore.Qt.Checked:
            topic = str(self.parent.analyseTopic)
            if 'compressed' in topic:
                self.sub = rospy.Subscriber(topic,CompressedImage,self.cam_cb)
            else:
                self.sub = rospy.Subscriber(topic,Image,self.cam_cb)
            self.canPublish = True
            self.ps = subprocess.Popen(['rosrun','image_view','image_view','image:='+'/%s'%self.name,'compressed'])
        else:
            self.canPublish = False
            self.sub.unregister()
            self.ps.terminate()

class Vision(QtGui.QWidget):

    def __init__(self):
        super(Vision, self).__init__()

        #Init ros
        rospy.init_node("Vision")
        rospy.loginfo("Launched")
        self.active = dict()
        self.img_topics = ["/bottomcam/camera/image_color",
        "/frontcam/camera/image_color",
        "bottomcam/camera/bag", "frontcam/camera/bag", 
        "bot_camera/camera/image_raw/compressed",
        "front_camera/camera/image_raw/compressed",
        "/bot_camera/camera/image_raw_jin", "/front_camera/camera/image_raw_jin"]
        self.currTopic = "/bottomcam/camera/bag"
        self.analyseTopic = "/bottomcam/camera/bag"
        self.threshTopic = "/bottomcam/camera/bag"
        self.threshlist = [ "/bottomcam/camera/image_color/compressed", 
                            "/frontcam/camera/image_color/compressed",
                            "/bottomcam/camera/bag",
                            "/finlaynorm/compressed", "log_chroma/compressed", 
                            "/iace/compressed", "grayworld_modified/compressed", 
                            "/naive_fusionmap/compressed", "/perfectnorm/compressed" ,
                            "/enhance_tan/compressed", "/french_preprocess/compressed"]

        """List of algos"""
        self.finlaynorm = Unit('finlaynorm',self)
        self.logchroma  = Unit('log_chroma',self)
        self.iace  = Unit('iace',self)
        self.grayworld = Unit('grayworld_modified',self)
        self.fusion  = Unit('naive_fusionmap',self)
        self.perfectnorm = Unit('perfectnorm',self)
        self.deilluminate = Unit('deilluminate',self)
        self.gamma1 = Unit('gamma1color',self)
        self.gamma2 = Unit('gamma2color',self)
        self.harris = Unit('harris',self)
        self.enhance_tan = Unit('enhance_tan',self)
        self.french = Unit('french_preprocess',self)
        """Analysis"""
        self.bgr = Analyze('getRGB',self)
        self.hsv = Analyze('getHSV',self)
        self.lab = Analyze('getLAB',self)
        """Robosub functions"""
        self.lane = Sub('findLane',self)
        self.allbins = Sub('findOverallBins',self)
        self.bincover = Sub('findCover',self)
        self.binpattern = Sub('findBins',self)
        self.findobj = Sub('identifyObject',self)
        self.train = Sub('findTrain',self)
        self.delorean = Sub('findDelorean',self)
        self.rail = Sub('findRail',self)
        self.railbox1 = Sub('getRailBox',self)
        self.railbox2 = Sub('getRailBox2',self)

        #Init UI
        self.initUI()

    def initUI(self):
        self.setupInfoGrid()
        self.stateSignal()
        self.setWindowTitle("VISION")
        self.show()
    

    def setupInfoGrid(self):
        '''Setup Grid Layout'''
        self.grid = QtGui.QGridLayout()
        self.grid.setVerticalSpacing(20)
        self.grid.setHorizontalSpacing(20)
        self.setLayout(self.grid)

        self.input = QtGui.QLabel("Enhancement")
        self.input2 = QtGui.QLabel("Analysis")
        self.input3 = QtGui.QLabel("Threshold")
        self.topics = createCombo(self.img_topics)
        self.topics2 = createCombo(self.img_topics)
        self.topics3 = createCombo(self.threshlist)
        self.save = createButton("Save")
        self.param1 = sliderInput("Threshold",0,255)

        '''Add Widgets'''
        self.grid.addWidget(self.topics,1,0) 
        self.grid.addWidget(self.input,1,1) 
        self.grid.addWidget(self.finlaynorm.box,2,0) 
        self.grid.addWidget(self.logchroma.box,2,1) 
        self.grid.addWidget(self.iace.box,2,2) 
        self.grid.addWidget(self.grayworld.box,2,3) 
        self.grid.addWidget(self.fusion.box,2,4) 
        self.grid.addWidget(self.french.box,2,5) 
        self.grid.addWidget(self.perfectnorm.box,3,0) 
        self.grid.addWidget(self.deilluminate.box,3,1) 
        self.grid.addWidget(self.gamma1.box,3,2) 
        self.grid.addWidget(self.gamma2.box,3,3) 
        self.grid.addWidget(self.harris.box,3,4) 
        self.grid.addWidget(self.enhance_tan.box,3,5) 
        self.grid.addWidget(self.topics2,4,0) 
        self.grid.addWidget(self.bgr.box,4,1) 
        self.grid.addWidget(self.hsv.box,4,2) 
        self.grid.addWidget(self.lab.box,4,3) 
        self.grid.addWidget(self.lane.box,5,0) 
        self.grid.addWidget(self.allbins.box,5,1) 
        self.grid.addWidget(self.bincover.box,5,2) 
        self.grid.addWidget(self.binpattern.box,5,3) 
        self.grid.addWidget(self.findobj.box,5,4) 
        self.grid.addWidget(self.train.box,6,0) 
        self.grid.addWidget(self.delorean.box,6,1) 
        self.grid.addWidget(self.rail.box,6,2) 
        self.grid.addWidget(self.railbox1.box,6,3) 
        self.grid.addWidget(self.railbox2.box,6,4) 
        self.grid.addWidget(self.topics3,7,0) 
        self.grid.addWidget(self.input3, 7,1) 
        
    def stateSignal(self):
        self.topics.activated[str].connect(self.topics_cb)
        self.topics2.activated[str].connect(self.topics2_cb)
        self.param1[0].valueChanged[int].connect(self.param1_sld_cb)


    '''Callbacks'''

    def param1_sld_cb(self,value):
    	self.param1[1].setText(str(value))

    def topics_cb(self,text):
    	rospy.loginfo("Topic selected")
    	self.currTopic = text
        for k,v in self.active.items():
            v.sub = rospy.Subscriber(str(text),Image,v.cam_cb)

    def topics2_cb(self,text):
    	rospy.loginfo("Topic selected")
    	self.analyseTopic = text

    def topics3_cb(self,text):
    	rospy.loginfo("Topic selected")
    	self.threshTopic = text

'''Utility'''

def sliderInput(name,min,max):
	edit = QtGui.QLineEdit()
	sld = QtGui.QSlider(QtCore.Qt.Horizontal)
	label = QtGui.QLabel(name)
	sld.setRange(min,max)
	return (sld,edit,label)

def createCombo(ls):
    box = QtGui.QComboBox()
    for i in ls:
        box.addItem(i)
    return box

def createButton(name):
    button = QtGui.QPushButton(name)
    return button

def horizontalLine():
    frame = QtGui.QFrame()
    frame.setFrameStyle(QtGui.QFrame.HLine)
    frame.setSizePolicy(QtGui.QSizePolicy.Minimum,QtGui.QSizePolicy.Expanding)
    return frame

def getTextEdit(name): 
    title = QtGui.QLabel(name) 
    info = QtGui.QLineEdit()
    return (title,info)

def getTextBox(name, data): 
    title = QtGui.QLabel(name+": ") 
    info = QtGui.QLabel(str(data))
    info.setStyleSheet('border:1px solid black')
    return (title,info)

def getCheckBox(name): 
    checkbox = QtGui.QCheckBox(name)
    return checkbox

if __name__ == "__main__":
    
    main_app = QtGui.QApplication(sys.argv)
    vision = Vision()
    sys.exit(main_app.exec_())
