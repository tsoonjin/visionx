#!/usr/bin/env python3
from collections import namedtuple

from detectors.lane_detector import LaneDetector
from utils import *
from configs.base_config import BaseConfig
from rosbridge.base_comm import BaseComm

class Disengage(GenericState):
    def __init__(self):
        self.transitions = {'static':'JUST OBSERVING', 'activated':'SEARCHING LANE'}
        self.sleep_duration = 5
        super(Disengage, self).__init__('DISENGAGE', self.transitions)

    @start_time
    def execute(self, userdata):
        if userdata.static:
            changeState('static')
        while isDormant:
            rospy.sleep(rospy.Duration(self.sleep_duration))
            changeState('activated')

class SearchLane(GenericState):
    def __init__(self):
        self.transitions = {'detected':'CENTERING TO LANE'}
        super(Disengage, self).__init__('SEARCHING LANE', self.transitions)

    @start_time
    def execute(self, userdata):
        if userdata.static:
            changeState('static')
        while isDormant:
            rospy.sleep(rospy.Duration(self.sleep_duration))
            changeState('activated')

def setupStates(comm):
    states_list = [Disengage(), Static(), SearchLane()]
    
if __name__ == "__main__":
    config = BaseConfig('lane', LaneDetector())
    comm = BaseComm(config)
    states_list = setupStates(comm)
    startTask(comm, states_list)
