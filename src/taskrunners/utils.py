#!/usr/bin/env python
"""Contains classes and reuseable functions to setup a taskrunner using SMACH state 
machine library
"""
import abc
import time
import signal

import rospy
import smach
import smach_ros

'''SMACH'''

def _initIntroServer(server_name, sm, path):
    """Initiazes introspection server"""
    introServer = smach_ros.IntrospectionServer(server_name, sm, path)
    introServer.start()

def _initSMACH(self, comm, states_list, outcomes_list):
    """Initializes State Machine container and add states
    Args:
        states: list of states that will be added 
        outcomes: list of possible outcomes 
    Returns:
        sm: Smach state machine instance
    """
    sm = smach.StateMachine(outcomes=outcomes_list)
    sm.userdata.comm = comm
    #Adding States from list 
    with sm:
        for state in states_list:
            smach.StateMachine.add(state.name, state, transitions=state.transitions)
    return sm

def _executeTask(self, comm, states_list, outcomes_list):
    """Initializing ROS node and start state machine
    Args:
        comm: Task specific communication module
    """
    sm = initSMACH(states_list, outcomes_list)
    initIntroServer(comm.name, sm, '/MISSION/{}'.format(comm.name.upper()))
    try:
        sm.execute()
    except Exception as e:
        rospy.logerr(str(e))
    finally:
        rospy.signal_shutdown('{} ended successfully'.format(comm.name))
    
def setupUserdata(self, userdata, comm):
    userdata.output = comm.
    
'''ROS initialization'''
def startTask(self, comm, states_list, outcomes_list):
    #Handle Ctrl+C 
    signal.signal(signal.SIGINT, comm.handleInterupt)
    #Initialize ROS node
    rospy.init_node(comm.name)
    _executeTask(comm, states_list, outcomes_list)

class Disengage(smach.State):
    """Standby while waiting for request from mission planner"""
    def __init__(self, outcomes_list):
        smach.State.__init__(self, outcomes=outcomes_list)
        self.start_time = None

    def execute(self, userdata):
        #Marks start of the state
        self.start_time = time.time()

        if self.comm.isKilled:
            return 'aborted'

        if self.comm.debug == 2:
            self.comm.isAlone = False
            self.comm.missionBridge("/bins/")
            self.comm.register()
            self.comm.debug = 1 #Reset to prevent multiple registration

        while not self.comm.isStart:
            if self.comm.isKilled: 
                return 'aborted'
            rospy.sleep(rospy.Duration(0.1))

        self.comm.time = time.time()        #Start timer for task
        if self.comm.debug <= 2:
            self.comm.startPID()
        return 'started'
