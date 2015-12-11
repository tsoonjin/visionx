#!/usr/bin/env python
import time

import rospy
import smach
import smach_ros

class BaseState(smach.State):
    def __init__(self):
        self.start_time = time.time() 
