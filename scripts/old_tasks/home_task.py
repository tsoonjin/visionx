#!/usr/bin/env python
import rospy
import sys
import time
from os.path import dirname

sys.path.append((dirname(dirname(__file__))))
import smach
import smach_ros
import time
from home_vision import HomeDetector
from home_comm import HomeComms

from utils.common import *

class Disengage(smach.State):

    def __init__(self, homer):
        smach.State.__init__(self, outcomes=['lane','started', 'aborted', 'done', 'drop','redo'])
        self.homer = homer
        self.comm = homer.comm

    def execute(self, userdata):
        #Wait for mission planner

        if self.comm.isKilled:
            return 'aborted'

        if self.comm.debug == 2:
            self.comm.isAlone = False
            self.comm.missionBridge("/pickup/")
            self.comm.register()
            self.comm.debug = 1 #Reset to prevent multiple registration

        while not self.comm.isStart:
            if self.comm.isKilled: 
                return 'aborted'
            rospy.sleep(rospy.Duration(0.1))

        self.comm.time = time.time()        #Start timer
        if self.comm.debug <= 2:
            self.comm.startPID()
        rospy.logerr("Captured: %d"%self.comm.captured)

        if self.comm.captured == 4:
            return 'lane'
        if self.comm.captured == 2:
            return 'drop'
        else:
            return 'started'


class Pickup(smach.State):

    def __init__(self, homer):
        smach.State.__init__(self, outcomes=['detected', 'aborted', 'retry', 'done', 'drop'])
        self.homer = homer
        self.comm = homer.comm
        self.center_depth = 2.6
        self.pickup_depth = 2.0
        self.pickup_area = 0.1
        self.surface_depth = 1.5
        self.doa = None
        self.count = 0
        self.timeout = 0
        self.fixedDOA = None
        self.detected = -1
        self.wait = 0

    def execute(self, userdata):

        self.detected = -1
        self.comm.detector = VUtil.detectGeneral
        rospy.logerr("IDENTIFYING OBJECT")
        rospy.sleep(rospy.Duration(0.5))

        if self.comm.captured == 0:
            self.comm.change_frequency()

        #Wait 10 iterations to confirm identity of object 
        self.comm.sendMovement(depth=1.2)
        for i in xrange(10):
            data = self.comm.outData.data
            if data['detected']:
                rospy.logwarn(data['object'])
                self.detected += data['object']
            rospy.sleep(rospy.Duration(0.5))


        self.wait = 0

        if self.detected >= 0:
            self.comm.detector = VUtil.findTrain
        elif self.detected < 0:
            self.comm.detector = VUtil.findDelorean
            self.pickup_area = 0.07
        else:
            self.comm.detector = VUtil.findDelorean

        rospy.logerr(self.detected)
        self.comm.detected = self.detected
        rospy.logerr("CENTERING TOWARDS: %s" % ("Train" if self.detected > 0 else "Delorean"))
        rospy.sleep(rospy.Duration(0.5))

        #self.comm.detector = VUtil.findTrain
        rospy.sleep(rospy.Duration(0.5))
        while not self.comm.isKilled:

            data = self.comm.outData.data
            if data['detected']:
                deltaX = data['dxy'][0]
                deltaY = data['dxy'][1]
                rospy.logwarn("dx: %.2f dy: %.2f" % (deltaX,deltaY))
                if abs(deltaX) <= 0.1 and abs(deltaY) <= 0.1:
                    #Align perpendicularly to object
                    while self.comm.outData.data['angle'] == 666:
                        rospy.sleep(rospy.Duration(0.5))
                    currDoa = self.comm.outData.data['angle']
                    rospy.logerr(str(currDoa))
                    self.comm.sendMovement(turn=currDoa)
                    rospy.sleep(rospy.Duration(0.5))

                    '''
                    #Wait 10 iterations to get heading of object
                    for i in xrange(5):
                        if self.comm.outData.data['detected']:
                            self.comm.direction += self.comm.outData.data['heading']
                        rospy.sleep(rospy.Duration(0.3))

                    '''
                    self.fixedDOA = self.comm.heading
                    rospy.logwarn("Setting fixed heading to %f"%self.fixedDOA)
                    heading = 'L' if self.comm.direction < 0 else 'R'
                    heading = 'L'
                    rospy.logwarn("HEADING: %s" % heading)
                    self.wait = 0
                    break
                self.comm.sendMovement(forward=deltaY*1.0, sidemove=deltaX*3.0, wait=False)
                
        if heading is 'R':
            rospy.logwarn("Turning to ensure object points left")
            self.comm.sendMovement(turn=180)
            rospy.sleep(rospy.Duration(1.0))
            self.fixedDOA = self.comm.heading
            rospy.logwarn("Setting fixed heading to %f"%self.fixedDOA)


        while not self.comm.isKilled:

            data = self.comm.outData.data
            if data['detected']:
                deltaX = data['dxy'][0]
                deltaY = data['dxy'][1]
                rospy.logwarn("dx: %.2f dy: %.2f" % (deltaX,deltaY))
                
                if abs(deltaX) <= 0.05 and abs(deltaY) <= 0.05:
                    break
                self.comm.sendMovement(forward=deltaY*1.0, sidemove=deltaX*3.0, wait=False)

        while not self.comm.isKilled:

            if self.comm.outData.data['detected']:
                deltaX = self.comm.outData.data['dxy'][0]
                deltaY = self.comm.outData.data['dxy'][1]
                area = self.comm.outData.data['area']
                '''
                offset = 1/(area/self.pickup_area)
                deltaX = offset*deltaX
                deltaY = offset*deltaY
                '''
                rospy.logwarn("dx: %.2f dy: %.2f a: %.2f" % (deltaX,deltaY,area))
                if area > self.pickup_area or self.comm.depth > self.pickup_depth:
                    '''
                    while not self.comm.isKilled:
                        data = self.comm.outData.data
                        if data['detected']:
                            deltaX = data['dxy'][0]
                            deltaY = data['dxy'][1]
                            rospy.logwarn("dx: %.2f dy: %.2f" % (deltaX,deltaY))
                            if abs(deltaX) <= 0.05 and abs(deltaY) <= 0.05:
                                break
                            self.comm.sendMovement(forward=deltaY*0.5, sidemove=deltaX*2.5, turn=self.fixedDOA, absolute=True, wait=False)
                            '''
                    rospy.logerr("DIVING DOWN TO GRAB")

                    for i in xrange(30):
                        self.comm.sendMovement(depth=self.comm.depth+0.2,turn=self.fixedDOA, absolute=True, wait=False,duration=0.8)
                    self.comm.grab()
                    rospy.logwarn("Enough sitting")
                    break
                else:
                    self.comm.sendMovement(forward=deltaY*1.5, sidemove=deltaX*1.5, depth=self.comm.depth+0.1,turn=self.fixedDOA,wait=False,absolute=True, duration=0.6)

            else:
                self.timeout += 1

        rospy.logerr("GRABBING")
        rospy.sleep(rospy.Duration(0.5))

        rospy.logerr("SURFACING")
        rospy.sleep(rospy.Duration(0.5))

        while self.comm.depth > self.surface_depth:
            self.comm.sendMovement(depth=self.comm.depth-0.3, wait=False, duration=3.0)

        self.comm.sendMovement(depth=0.0)
        rospy.logwarn("SENT 0")
        self.comm.sendMovement(depth=-0.5, wait=False, duration=10.0)
        rospy.sleep(rospy.Duration(0.5))
        self.comm.sendMovement(depth=0)
        rospy.logerr("Captured: %d"%self.comm.captured)
        if self.comm.captured == 1:
            self.comm.captured += 1
            self.comm.call2D()
            return 'drop'
        else:
            return 'detected'

class FindLane(smach.State):

    def __init__(self, homer):
        smach.State.__init__(self, outcomes=['detected', 'aborted', 'retry', 'done'])
        self.homer = homer
        self.comm = homer.comm
        self.found = 0

    def execute(self, userdata):


        self.comm.detector = VUtil.findLane

        '''
        @TODO Need to implement a search pattern to identify lane
        '''

        rospy.logerr("SEARCHING FOR LANE")
        rospy.sleep(rospy.Duration(0.5))

        self.comm.sendMovement(depth=0.5)

        while not self.comm.isKilled:
            if self.comm.ping:
                doa = self.comm.ping['doa']
                self.comm.sendMovement(turn=doa)
                break

        while not self.comm.isKilled:

            data = self.comm.outData.data
            if data['detected']:
                self.found += 1
                deltaX = self.comm.outData.data['dxy'][0]
                deltaY = self.comm.outData.data['dxy'][1]
                angle = self.comm.outData.data['angle']
                rospy.logwarn("dx: %.2f dy: %.2f t: %d" % (deltaX,deltaY,int(angle)))

                if abs(deltaX) <= 0.1 and abs(deltaY) <= 0.1:
                    angle = self.comm.outData.data['angle']
                    rospy.logwarn("TURNING TO LANE")
                    self.comm.sendMovement(turn=angle)
                    return 'detected'

                if abs(deltaX) != 0.2 and abs(deltaY) != 0.2:
                    self.comm.sendMovement(forward=1.5*deltaY, sidemove=2.0*deltaX, wait=False)

            elif self.found < 2:
                rospy.logwarn("MOVING FORWARD")
                self.comm.sendMovement(forward=0.5, wait=False)


        return 'aborted'


class DropRail(smach.State):

    def __init__(self, homer):
        smach.State.__init__(self, outcomes=['found_rail', 'aborted', 'retry', 'done'])
        self.homer = homer
        self.comm = homer.comm
        self.rail_depth = 3.0
        self.drop_depth = 2.8
        self.count = 0
        self.turn = False
        self.fixedDOA = None

    def execute(self, userdata):

        self.comm.detector = VUtil.findRail
        rospy.logerr("CENTERING TO RAIL")
        rospy.sleep(rospy.Duration(1.0))

        self.comm.sendMovement(depth=1.0)
        while not self.comm.isKilled:

            if self.comm.outData.data['detected']:

                rospy.sleep(rospy.Duration(0.5))
                deltaX = self.comm.outData.data['dxy'][0]
                deltaY = self.comm.outData.data['dxy'][1]
                rospy.logwarn("dx: %.2f dy: %.2f" % (deltaX,deltaY))

                while abs(deltaX) > 0.05 and abs(deltaY) > 0.05:
                    #Orientate towards correct direction 
                    deltaX = self.comm.outData.data['dxy'][0]
                    deltaY = self.comm.outData.data['dxy'][1]
                    angle = self.comm.outData.data['angle']
                    self.comm.sendMovement(forward=deltaY*2.0, sidemove=deltaX*3.5, wait=False)

                while self.comm.outData.data['angle'] == 666:
                    angle = self.comm.outData.data['angle']

                if not self.turn:
                    angle = self.comm.outData.data['angle']
                    self.comm.sendMovement(turn=angle) #Align with track
                    self.turn = True
                    rospy.logwarn("TURNING")

                if self.comm.captured != 2:
                    self.comm.sendCentered()

                #rotate = 90 if self.comm.direction < 0 else -90
                rotate = 90
                rospy.logwarn("GET CORRECT ORIENTATION")
                self.comm.sendMovement(turn=rotate)
                break

            else:
                self.comm.sendMovement(forward=0.5, wait=False)

        rospy.logwarn("Center after turning")
        self.fixedDOA = self.comm.heading
        while abs(deltaX) > 0.1 and abs(deltaY) > 0.1:
            #Orientate towards correct direction 
            deltaX = self.comm.outData.data['dxy'][0]
            deltaY = self.comm.outData.data['dxy'][1]
            self.comm.sendMovement(forward=deltaY*2.0, sidemove=deltaX*3.5, wait=False)

        self.comm.detector = VUtil.detectSmallSquare
        rospy.logerr("DIVING TO RAIL BOX")
        rospy.sleep(rospy.Duration(0.5))

        while self.comm.depth < self.rail_depth:

            if self.comm.isKilled:
                return 'aborted'

            if self.comm.outData.data['detected']:
                deltaX = self.comm.outData.data['dxy'][0]
                deltaY = self.comm.outData.data['dxy'][1]
                rospy.logwarn("dx: %.2f dy: %.2f" % (deltaX,deltaY))
                if abs(deltaY) != 1 and abs(deltaX) != 1:
                    self.comm.sendMovement(forward=deltaY*2.0, sidemove=deltaX*3.5, turn=self.fixedDOA, absolute=True,depth=self.comm.depth+0.1,wait=False)
            else:
                self.comm.sendMovement(depth=self.comm.depth+0.1, wait=False)

        rospy.logerr("DROPPING")
        self.comm.drop()
        self.comm.captured += 1
        rospy.logerr("Captured: %d"%self.comm.captured)
        self.comm.sendMovement(depth=0.3)
        if self.comm.captured == 1:
            self.comm.callAcoustics()
        elif self.comm.captured == 3:
            self.comm.taskComplete()
        return 'found_rail'

if __name__ == '__main__':
    rospy.init_node('homer')
    mode = rospy.get_param('~mode', 2)
    print(mode)
    homer = HomeDetector()
    homer.comm = HomeComms('homer', homer.detect, mode)
    rospy.loginfo("Homer loaded!")

    sm = smach.StateMachine(outcomes=['task_complete', 'aborted'])
    with sm:
        smach.StateMachine.add('DISENGAGE', Disengage(homer),
                               transitions={'started' : 'PICKUP',
                                   'aborted' : 'aborted', 'done':'task_complete',
                                   'drop':'DROPRAIL','redo':'DISENGAGE',
                                   'lane':'FINDLANE'})

        smach.StateMachine.add('PICKUP', Pickup(homer),
                               transitions={'detected' : 'FINDLANE',
                                   'aborted' : 'aborted', 'retry':'PICKUP', 
                                   'done':'task_complete', 'drop':'DISENGAGE'})

        smach.StateMachine.add('FINDLANE', FindLane(homer),
                               transitions={'detected' : 'DROPRAIL',
                                   'aborted' : 'aborted', 'retry':'FINDLANE', 
                                   'done':'task_complete'})

        smach.StateMachine.add('DROPRAIL', DropRail(homer),
                               transitions={'found_rail' : 'DISENGAGE',
                                   'aborted' : 'aborted', 'retry':'DROPRAIL',
                                   'done':'task_complete'})

    introServer = smach_ros.IntrospectionServer('homer_server',
                                                sm,
                                                '/MISSION/HOME')
    introServer.start()

    try:
        sm.execute()
    except Exception as e:
        rospy.logerr(str(e))
    finally:
        rospy.signal_shutdown("home task ended")
