#!/usr/bin/env python3
from detectors.lane_detector import LaneDetector
from utils import *
from configs.base_config import BaseConfig
from rosbridge.base_comm import BaseComm


def setup(name):
    """Setup configuration and communication of ROS node
    Args:
        name: name of ROS node
    Returns:
        comm: object that handles communication with ROS
    """
    config = BaseConfig(name, LaneDetector())
    rospy.init_node(lane_config.name)
    comm = BaseComm(lane_config)
    return comm

def executeTask(comm):
    states_list = 
    

if __name__ == "__main__":
    comm = setup('lane')
    startTask(
