#!/usr/bin/env python

import rospy
import sys
import subprocess
import signal
import os
import actionlib
import dynamic_reconfigure.client
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Vector3Stamped
from bbauv_msgs.msg import *
from bbauv_msgs.srv import *
from PyQt4 import QtGui,QtCore

class Info(QtGui.QMainWindow):

    def __init__(self):
        super(Info, self).__init__()
        signal.signal(signal.SIGTERM,self.sigHandler)

        rospy.init_node("GUI")
        rospy.loginfo("Launched")
        self.info = {'yaw':None, 'depth':None, 'pressure':None, 'batt1':0.0,
                'batt2':0.0, 'abs_x': None, 'abs_y': None, 'setpoint':"", 'acoustic':(0.0,0.0)}

        #Setup controller
        self.set_controller_request = rospy.ServiceProxy('set_controller_srv',set_controller)
        self.locomotion_mode_request = rospy.ServiceProxy('locomotion_mode_srv',locomotion_mode)
        self.client = actionlib.SimpleActionClient('LocomotionServer', ControllerAction)
        self.controller_client = dynamic_reconfigure.client.Client('/Controller')

        #Front and bottomcam
        self.front_topic = rospy.get_param('~front','/frontcam/camera/image_color')
        self.bot_topic = rospy.get_param('~bottom','/bottomcam/camera/image_color')
        self.front_ps = None
        self.bot_ps = None
        
        self.register()
        #Init UI
        self.initUI()

    def sigHandler(self):
        rospy.signal_shutdown("Aborted")

    def initUI(self):
        self.setupInfoGrid()
        self.stateSignal()
        self.setWindowTitle("Gooey")
        self.show()
    

    def register(self):
        rospy.Subscriber('/navigation/RPY', Vector3Stamped, self.compass_cb)
        rospy.Subscriber('/depth', depth , self.depth_cb)
        rospy.Subscriber('/earth_odom', Odometry , self.earth_cb)
        rospy.Subscriber("/battery1_status", Battery, self.batt1_cb)
        rospy.Subscriber("/battery2_status", Battery, self.batt2_cb)
        rospy.Subscriber("/controller_setpoints", controller, self.controller_cb)

    def compass_cb(self,data):
        self.info['yaw'] = round(data.vector.z,2)
        rospy.sleep(rospy.Duration(0.5))
        self.yaw[1].setText(str(self.info['yaw']))

    def depth_cb(self,data):
        self.info['depth'] = round(data.depth,2)
        self.info['pressure'] = round(data.pressure,2)
        rospy.sleep(rospy.Duration(0.5))
        self.depth[1].setText(str(self.info['depth']))
        self.pressure[1].setText(str(self.info['pressure']))

    def earth_cb(self,data):
        self.info['abs_x'] = round(data.pose.pose.position.x,2)
        self.info['abs_y'] = round(data.pose.pose.position.y,2)
        rospy.sleep(rospy.Duration(0.5))
        self.abs_x[1].setText(str(self.info['abs_x']))
        self.abs_y[1].setText(str(self.info['abs_y']))

    def batt1_cb(self,data):
        self.info['batt1'] = round(data.cell6,2)
        rospy.sleep(rospy.Duration(0.5))
        self.batt1[1].setText(str(self.info['batt1']))

    def batt2_cb(self,data):
        self.info['batt2'] = round(data.cell6,2)
        rospy.sleep(rospy.Duration(0.5))
        self.batt2[1].setText(str(self.info['batt2']))

    def controller_cb(self,data):
        self.info['setpoint'] = (data.heading_setpoint, data.forward_setpoint, 
                data.sidemove_setpoint, data.depth_setpoint)
        rospy.sleep(rospy.Duration(0.5))
        self.setpoint[1].setText(round(data.heading_setpoint,2)+" , "+round(data.forward_setpoint,2)+" , "
                +round(data.sidemove_setpoint,2)+" , "+round(data.depth_setpoint,2))

    def setupInfoGrid(self):
        '''Setup Grid Layout'''
        self.mainWidget = QtGui.QWidget()
        self.mainWidget.setStyleSheet('background-color:black')
        grid = QtGui.QGridLayout()
        grid.setVerticalSpacing(20)
        grid.setHorizontalSpacing(20)
        self.mainWidget.setLayout(grid)

        '''Setup Button'''
        self.disable_pid = createButton('Disable PID')
        self.caliberate_depth = createButton('Caliberate Depth')
        self.send_goal = createButton('Send')

        '''Setup text box'''
        self.yaw = getTextBox('Yaw', self.info['yaw'])
        self.depth = getTextBox('Depth', self.info['depth'])
        self.pressure = getTextBox('Pressure', self.info['pressure'])
        self.batt1 = getTextBox('Batt1', self.info['batt1'])
        self.batt2 = getTextBox('Batt2', self.info['batt2'])
        self.abs_x = getTextBox('X', self.info['abs_x'])
        self.abs_y = getTextBox('Y', self.info['abs_y'])
        self.setpoint = getTextBox('Setpoint(H/F/SM/D)', self.info['setpoint'])
        self.acoustic = getTextBox('Acoustic', self.info['acoustic'])
        self.telemetry = getTextEdit('INFO','red')
        self.control = getTextEdit('CONTROL','red')
        self.goal_f = getEqBox("F", "")
        self.goal_h = getEqBox("H", "")
        self.goal_sm = getEqBox("SM", "")
        self.goal_d = getEqBox("D", "")

        '''Setup Check Box'''
        self.front = getCheckBox('Front')
        self.bot = getCheckBox('Bot')

        grid.addWidget(self.telemetry[0],0,0)
        grid.addWidget(self.yaw[0],1,0)
        grid.addWidget(self.yaw[1],1,1)
        grid.addWidget(self.depth[0],2,0)
        grid.addWidget(self.depth[1],2,1)
        grid.addWidget(self.pressure[0],3,0)
        grid.addWidget(self.pressure[1],3,1)
        grid.addWidget(self.abs_x[0],1,2)
        grid.addWidget(self.abs_x[1],1,3)
        grid.addWidget(self.abs_y[0],2,2)
        grid.addWidget(self.abs_y[1],2,3)
        grid.addWidget(self.acoustic[0],3,2)
        grid.addWidget(self.acoustic[1],3,3)
        grid.addWidget(self.batt1[0],5,0)
        grid.addWidget(self.batt1[1],5,1)
        grid.addWidget(self.batt2[0],5,2)
        grid.addWidget(self.batt2[1],5,3)
        grid.addWidget(self.setpoint[0],4,0)
        grid.addWidget(self.setpoint[1],4,1)
        grid.addWidget(self.front,4,2)
        grid.addWidget(self.bot,4,3)
        grid.addWidget(self.control[0],6,0)
        grid.addWidget(self.goal_f[0],7,0)
        grid.addWidget(self.goal_f[1],7,1)
        grid.addWidget(self.goal_h[0],7,2)
        grid.addWidget(self.goal_h[1],7,3)
        grid.addWidget(self.goal_sm[0],7,4)
        grid.addWidget(self.goal_sm[1],7,5)
        grid.addWidget(self.goal_d[0],7,6)
        grid.addWidget(self.goal_d[1],7,7)
        grid.addWidget(self.disable_pid,6,2)
        grid.addWidget(self.caliberate_depth,6,3)
        grid.addWidget(self.send_goal,7,8)
        self.setCentralWidget(self.mainWidget)
        
    def sendMovement(self,forward=0.0, sidemove=0.0, turn=None, depth=None, absolute=False, wait=True):
        resp = self.set_controller_request(True, True, True, True, True, True,
                                           False, False, False,False)
        depth = depth if depth else self.info['depth']
        if turn is None:
            turn = self.info['yaw']
            goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
        else:
            if absolute:
                goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
            else:
                turn = (turn+self.info['yaw'])%360 
                goal = ControllerGoal(forward_setpoint=forward, heading_setpoint=turn, sidemove_setpoint=sidemove, depth_setpoint=depth)
        self.client.send_goal(goal)

    def stateSignal(self):
        self.front.stateChanged.connect(self.front_checkbox_cb)
        self.bot.stateChanged.connect(self.bot_checkbox_cb)
        self.disable_pid.clicked.connect(self.disable_pid_cb)
        self.caliberate_depth.clicked.connect(self.caliberate_depth_cb)
        self.send_goal.clicked.connect(self.send_goal_cb)

    '''Callbacks'''
    def caliberate_depth_cb(self,state):
        rospy.loginfo("Caliberated depth")
        params = {'depth_offset': 0}
        config = self.controller_client.update_configuration(params)
        rospy.sleep(1.0)

    def send_goal_cb(self,state):
        rospy.loginfo("Goal sent")
        f = float(self.goal_f[1].text()) if self.goal_f[1].text() != "" else 0.0
        d = float(self.goal_d[1].text()) if self.goal_d[1].text() != "" else self.info['depth']
        h = float(self.goal_h[1].text()) if self.goal_h[1].text() != "" else self.info['yaw']
        sm = float(self.goal_sm[1].text()) if self.goal_sm[1].text() != "" else 0.0
        print("F: %.2f, H: %.2f, SM: %.2f, D: %.2f" % (f,h,sm,d))
        #sendMovement(forward=f,depth=d,sidemove=sm,turn=h)

    def disable_pid_cb(self,state):
        rospy.loginfo("PID disabled")
        self.set_controller_request(False,False,False,False,False,
                False,False,False,False,False)

    def front_checkbox_cb(self, state):
        if state == QtCore.Qt.Checked:
            self.front_ps = subprocess.Popen(['rosrun','image_view','image_view','image:='+self.front_topic,'compressed'])
        else:
            self.front_ps.terminate()

    def bot_checkbox_cb(self, state):
        if state == QtCore.Qt.Checked:
            self.bot_ps = subprocess.Popen(['rosrun','image_view','image_view','image:='+self.bot_topic,'compressed'])
        else:
            self.bot_ps.terminate()


'''Utility'''

def createButton(name):
    button = QtGui.QPushButton(name)
    button.setStyleSheet('background-color:white; color:black')
    return button

def horizontalLine():
    frame = QtGui.QFrame()
    frame.setFrameStyle(QtGui.QFrame.HLine)
    frame.setSizePolicy(QtGui.QSizePolicy.Minimum,QtGui.QSizePolicy.Expanding)
    return frame

def getTextEdit(name,color='white'): 
    title = QtGui.QLabel(name) 
    info = QtGui.QLineEdit()
    title.setStyleSheet('color:'+color)
    info.setStyleSheet('color:'+color)
    return (title,info)

def getEqBox(name, data): 
    title = QtGui.QLabel(name+": ") 
    info = QtGui.QLineEdit(str(data))
    title.setStyleSheet('color:white')
    info.setStyleSheet('color:white; border:1px solid white')
    return (title,info)

def getTextBox(name, data): 
    title = QtGui.QLabel(name+": ") 
    info = QtGui.QLabel(str(data))
    title.setStyleSheet('color:white')
    info.setStyleSheet('color:white; border:1px solid white')
    return (title,info)

def getCheckBox(name): 
    checkbox = QtGui.QCheckBox(name)
    checkbox.setStyleSheet('color:white')
    return checkbox

if __name__ == "__main__":
    
    main_app = QtGui.QApplication(sys.argv)
    info_win = Info()
    sys.exit(main_app.exec_())
