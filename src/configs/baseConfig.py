#!/usr/bin/env python
from collections import namedtuple
import rospy
from geometry_msgs.msg import Vector3Stamped

SubscribeArg = namedtuple('SubscribeArg', 'topic msg_type')

class BaseConfig(object):
    
    systemTopics = {'heading':SubscribeArg(topic='/navigation/RPY', msg_type=Vector3Stamped)}

if __name__ == "__main__":
    baseConfig = BaseConfig()
