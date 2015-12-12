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
    sm.userdata = comm      #set userdata to comm object to share data between states
    initIntroServer(comm.name, sm, '/MISSION/{}'.format(comm.name.upper()))
    try:
        sm.execute()
    except Exception as e:
        rospy.logerr(str(e))
    finally:
        rospy.signal_shutdown('{} ended successfully'.format(comm.name))
    
'''ROS related'''

def startTask(self, comm, states_list, outcomes_list):
    #Handle Ctrl+C and kill
    signal.signal(signal.SIGINT, comm.handleInterupt)
    signal.signal(signal.SIGTERK, comm.handleInterupt)
    #Initialize ROS node
    rospy.init_node(comm.name)
    _executeTask(comm, states_list, outcomes_list)

def isDormant(self, comm):
    return not comm.alone and not comm.activated

'''Movement code'''

def centerToObject(comm, mult=(1.5, 2.0), limit=0.05):
    """Center to object of interest after detection
    Args:
        coeff: multiplier to forward and sidemove goals
        limit: error between centroid of object and center before terminating
    """
    while True:
        if comm.output.detected:
            if(abs(comm.output.dx) <= limit and abs(comm.output.dy) <= limit):
                break
            else:
                comm.move(f=mult[1]*comm.output.dy, sm=mult[0]*comm.output.dx, duration=0.5)

def searchForward(comm, limit=2):
    """Move forward slowly while detecting object of interest
    Args:
        limit: number of detection required before termination to remove false positive
    """
    count = 0
    while True:
        if count is limit:                     #stop searching after fulfilling limit
            break
        elif comm.output.detected:
            count += 1
        else:
            comm.move(f=1.0, duration=0.5)     #search forward slowly

'''Generic states'''

class Disengage(smach.State):
    """Standby while waiting for request from mission planner
        outcomes_list: ['started', 'static_mode']
    """
    def __init__(self, outcomes_list):
        self.outcomes_list = outcomes_list
        self.start_time = None
        self.sleep_duration = rospy.Duration(0.5)
        smach.State.__init__(self, outcomes=outcomes_list)

    def execute(self, userdata):
        #Marks start of the state
        self.start_time = time.time()
        if userdata.static:
            return 'static_mode'     #only run vision algorithm without any movement

        while isDormant(userdata): 
            rospy.sleep(self.sleep_duration)

        self.start_time = time.time()        #Start timer for task
        return 'started'

class StaticMode(smach.State):
    """See-only mode where the vehicle is moved around by diver or manually to 
    simulate specific scenario to test robustness of vision algorithm
    """
    def __init__(self, outcomes_list):
        self.outcomes_list = outcomes_list
        smach.State.__init__(self, outcomes=outcomes_list)
    
    def execute(self, userdata):
        while not userdata.completed:
            rospy.sleep(rospy.Duration(5.0))
        return 'completed'
