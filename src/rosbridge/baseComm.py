#!/usr/bin/env python
import signal 

import rospy
import cv2
import numpy as np
import actionlib

from visionx.srv import *
from configs import *

class BaseComm(object):
    """Main communication class that interacts with main system"""

    def __init__(self, config):
        #Trap signal
        signal.signal(signal.SIGINT, self.handleInterupt)
        #Configuration file 
        self.config = config
        #State of the taskrunner
        self.state = {'activated':False, 'preempted':False, 'completed':False,
                       'static':False, 'alone':False}
        #Task specific topics
        self.visionServerTopic = "/{}/mission_to_vision".format(config.name);
        self.missionServerTopic = "/{}/vision_to_mission".format(config.name);

        #Sensor data 
        self.data = {'heading':None, 'depth':None, 'frontcam':None, 'bottomcam':None}
        #Timeout parameters
        self.navTimeout = 5
        #Movement parameters
        self.move_params = {'xcoeff':2.5, 'ycoeff':3.0}

    '''Initilization'''

    def initSub(self):
        sys_msgs = self.config.sys_msgs       
        #Heading subscriber
        rospy.Subscriber(sys_msgs['heading'].topic, sys_msgs['heading'].type, self.heading_cb)
        #Depth subscriber
        rospy.Subscriber(sys_msgs['depth'].topic, sys_msgs['depth'].type, self.depth_cb)
        #Frontcam subscriber
        rospy.Subscriber(sys_msgs['frontcam_raw'].topic, sys_msgs['frontcam_raw'].type, self.cam_cb)
        #Bottomcam subscriber
        rospy.Subscriber(sys_msgs['bottomcam_raw'].topic, sys_msgs['bottomcam_raw'].type, self.cam_cb)


    def initPub(self):
        sys_msgs = self.config.sys_msgs       
        #Vision result publisher
        rospy.Publisher(sys_msgs['vision_output'].topic, sys_msgs['vision_output'].type)

    def initService(self):
        """Initialize connections with mission planner"""
        #Initialize vision server and mission serviceproxy
        self.visionServer = rospy.Service(self.visionServerTopic, mission_to_vision, self.handleMission)
        self.sendMission = rospy.ServiceProxy(self.missionServerTopic, vision_to_mission)
        #Connect to mission planner
        self.sendMission.wait_for_service()

    def initNavigation(self):
        """Connect with action server in charge of vehicle navigation"""
        self.setNavServer = actionlib.SimpleActionClient(sys_msgs['navigation_server'].topic, 
                sys_msgs['navigation_server'].type)
        try:
            self.setNavServer.wait_for_server(timeout=self.navTimeout)
        except:
            pass 

    def initPID(self):
        self.setPIDServer = rospy.ServiceProxy(sys_msgs['pid'].topic, sys_msgs['pid'].type)
        #Turn on PID
        self.setPIDServer(forward=True, sidemove=True, heading=True, depth=True,    
                  pitch=True, roll=True, topside=False, navigation=False)

    def initAll(self):
        """Initializes all components related to ROS"""
        initService()
        initSub()
        initNavigation()
        if not self.state['static']:
            initPID()

    '''Callbacks'''
    
    def heading_cb(self, data):
        self.data['heading'] = data.vector.z

    def depth_cb(self, data):
        self.data['depth'] = data.depth

    def cam_cb(self, rosimg):
        pass

    def handleInterupt(self, signal, frame):
        self.state['preempted'] = True
        if self.setNavServer:
            self.setNavServer.cancel_all_goals()
        rospy.signal_shutdown("Interrupted")

    def handleMission(self, req):
        if req.start_request:
            self.state['activated'] = True
            return mission_to_visionResponse(start_response=True,
                                             abort_response=False)
        elif req.abort_request:
            self.state['preempted'] = True
            self.state['activated'] = False
            return mission_to_visionResponse(start_response=False,
                                             abort_response=True)

    '''Mission planner requests'''

    def sendTaskComplete(self):
        """Signal mission planner a task is completed"""
        if not self.state['alone']:
            self.sendMission(fail_request=False, task_complete_request=True,
                             task_complete_ctrl=controller(heading_setpoint=self.data['heading']))
        self.state['activated'] = False

    '''Navigation server requests'''
    
    def move(self,forward=0.0, sidemove=0.0, turn=None, depth=None, relative=True, wait=True, duration=0.3):
        depth = depth if depth else self.heading['depth']
        if turn is None:
            turn = self.data['heading']
        elif relative:
            turn = (turn+self.heading)%360 
        goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, 
                sidemove_setpoint=sidemove, depth_setpoint=depth)
        self.setNavServer.send_goal(goal)
        if  wait:
        	self.setNavServer.wait_for_result()
        else:
			self.setNavServer.wait_for_result(rospy.Duration(duration))

if __name__ == "__main__":
    baseComm = BaseComm();
