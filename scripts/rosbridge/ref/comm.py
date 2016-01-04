#!/usr/bin/env python

#ROS 
import rospy
import cv2
import numpy as np
import actionlib
from sensor_msgs.msg import Image,CompressedImage
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
#from controls.msg import *
#from controls.srv import *
#from sparton_imu.msg import *
#from explorer_dvl.msg import *


#Custom
from common import *
from state_test import Tester

#Python
import signal
import time

class Comms:

    def __init__(self, name, detector, debug = None):
        signal.signal(signal.SIGINT, self.handleTerm)

        #Parameters
        self.isStart = rospy.get_param('~start', False) #Activated when called by mission planner 
        self.isDone = rospy.get_param('~done', False) #Determines completion of task
        self.isKilled = rospy.get_param('~killed', False) #Determines completion of task
        self.task = name
        self.isWait = rospy.get_param('~wait', True)
        self.isAlone = True
        self.canPublish = False
        self.publisher = None
        self.detector = detector
        self.publish_topic = "/Vision/image_filter/"+name
        self.publish_topic_compressed = "vision/%s/compressed"%name
        self.bag = rospy.get_param('~bag', False)
        self.debug = debug
        self.isRegistered = False
        self.frames = VQueue()
        self.time = time.time()

        #Input from sensors
        self.depth = None
        self.heading = None
        self.img_topic = "/bottomcam/camera/image_color" 
    	self.outData = Info()
    	self.ping = None

        #config 
        self.xcoeff = 2.5
        self.ycoeff = 3.0
        '''
        1 -> use bag file 
        2 -> normal mode
        3 -> without mission planner
        '''

        if debug == 1:
	        self.img_topic = "/bottomcam/camera/bag" if not self.bag else self.bag
	        self.register()

        if debug == 3:
	        self.img_topic = "/bottomcam/camera/image_color" if not self.bag else self.bag
	        self.register()

    def register(self):
        self.canPublish = True
        self.move = actionlib.SimpleActionClient("LocomotionServer",
                                                         ControllerAction)
        self.setServer = rospy.ServiceProxy("/set_controller_srv", set_controller)

        try:
            rospy.loginfo("Waiting for LocomotionServer...")
            self.move.wait_for_server(timeout=rospy.Duration(5))
        except:
            rospy.loginfo("LocomotionServer timeout!")

        rospy.Subscriber('/navigation/RPY', Vector3Stamped, self.compass_cb)
        rospy.Subscriber('/depth', depth , self.depth_cb)
        rospy.Subscriber(self.img_topic, Image, self.cam_cb)
        rospy.Subscriber('/acoustics/pingu', pingu, self.ac_cb)
        self.publisher = rospy.Publisher(self.publish_topic, Image)
        self.visionfilter = rospy.Publisher(self.publish_topic_compressed, CompressedImage)
        self.commonfilter = rospy.Publisher('/camera_processed/compressed', CompressedImage)
        rospy.loginfo("Complete registration")
        


    def sendMovement(self,forward=0.0, sidemove=0.0, turn=None, depth=None, absolute=False, wait=True, duration=0.3):
        depth = depth if depth else self.depth
        if turn is None:
            turn = self.heading
            goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
        else:
            if absolute:
                goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
            else:
                turn = (turn+self.heading)%360 
                goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
		#rospy.loginfo("F: %f, SM: %f, T: %f, D: %f" % (forward,sidemove,turn,depth))
        self.move.send_goal(goal)
        if  wait:
        	self.move.wait_for_result()
        else:
			self.move.wait_for_result(rospy.Duration(duration))

    #Callbacks
    def ac_cb(self, data):
        self.ping = {'doa':data.doa, 'elevation': data.elevation, 'dist':data.distance}

    def depth_cb(self, data):
        self.depth = data.depth

    def compass_cb(self,data):
        self.heading = data.vector.z
        while self.heading is None and not self.isKilled:
	        rospy.sleep(rospy.Duration(1.0))

    def cam_cb(self,rosimg):
        cvimg = VUtil.rosimg2cv(rosimg)
        y,x = cvimg.shape[:2]
        cvimg = cv2.resize(cvimg, (x/2,y/2))
        blank = np.zeros_like(cvimg)
        self.frames.add(cvimg)
        self.outData = Info()
        if self.canPublish:
            out = self.detector(cvimg,self.outData.data,blank)
            #self.outData.draw(blank)
            if self.debug <= 2:
                self.commonfilter.publish(VUtil.writeCompressed(out))
            elif self.debug == 3:
                self.visionfilter.publish(VUtil.writeCompressed(out))

    def handleTerm(self,signal,frame):
        self.isKilled = True
        if self.move:
            self.move.cancel_all_goals()
        rospy.signal_shutdown("KILLED")

    #Interaction with mission planner 

    def taskComplete(self, heading=0.0):
        rospy.loginfo("Sending Complete request to mission planner")
        elapsed = self.timeleft(self.time)
        rospy.logerr("Total elapsed: " + str(elapsed))
        if not self.isAlone:
            self.toMission(fail_request=False, task_complete_request=True,
                           task_complete_ctrl=controller(
                               heading_setpoint=heading))
        self.canPublish = False
        self.isStart = False
        #self.move.cancel_all_goals()

    def callAcoustics(self):
        rospy.logerr("Calling Acoustics")
        if not self.isAlone:
            self.toMission(fail_request=False, task_complete_request=False, 
                    action=666)
        self.canPublish = False
        self.isStart = False

    def call2D(self):
        rospy.logerr("Calling Navigate 2D")
        if not self.isAlone:
            self.toMission(fail_request=False, task_complete_request=False, 
                    action=999)
        self.canPublish = False
        self.isStart = False
        
    def sendCentered(self):
        rospy.logerr("Sending Centered")
        if not self.isAlone:
            self.toMission(fail_request=False, task_complete_request=False, 
                    centered=True)
        self.isStart = False

    def handleSrv(self, req):
       
        if req.start_request:
            rospy.loginfo("Received Start Request")
            self.isKilled = False
            self.isStart = True
            self.canPublish = True
            return mission_to_visionResponse(start_response=True,
                                             abort_response=False)
        elif req.abort_request:
            rospy.loginfo("Received Abort Request")
            self.isKilled = True
            self.isStart = False
            return mission_to_visionResponse(start_response=False,
                                             abort_response=True)

    def missionBridge(self, name):
        rospy.loginfo("Setting up vision server")
        self.comServer = rospy.Service(name+"mission_to_vision",
                                       bbauv_msgs.srv.mission_to_vision,
                                       self.handleSrv)
        rospy.loginfo("Waiting mission planner request")
        self.toMission = rospy.ServiceProxy(name+"vision_to_mission",
                                            bbauv_msgs.srv.vision_to_mission)
        self.toMission.wait_for_service() #Waiting indefintely
        rospy.loginfo("Waiting for mission service")

    def deactivateVelocity(self):
        self.setServer(forward=True, sidemove=True, heading=True, depth=True,
                       pitch=True, roll=True, topside=False, navigation=False,
                       forward_vel=False, sidemove_vel=False)


    def activateVelocity(self):
        self.setServer(forward=False, sidemove=False, heading=True, depth=True,
                       pitch=True, roll=True, topside=False, navigation=False,
                       forward_vel=True, sidemove_vel=True)

    def startPID(self):
        self.setServer(forward=True, sidemove=True, heading=True, depth=True,    #turn pid on
                  pitch=True, roll=True, topside=False, navigation=False)

    def timeleft(self,start):
        duration = (time.time() - start)/60.0
        return duration
