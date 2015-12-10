#!/usr/bin/env python
from collections import namedtuple

import rospy
from sensor_msgs.msg import Image,CompressedImage
from geometry_msgs.msg import Vector3Stamped

from visionx.msg import depth


SubscribeArg = namedtuple('SubscribeArg', 'topic msg_type')

class BaseConfig(object):
    """
    Basic configuration that stores details pertaining to ROS related parameters and the task
    """

    #Details of system-wide subscribers used
    system_subscribers = {'heading':SubscribeArg(topic='/navigation/RPY', msg_type=Vector3Stamped),
                    'depth':SubscribeArg(topic='/depth', msg_type=depth),
                    'frontcam_compressed':SubscribeArg(topic='/frontcam/camera/image_color/compressed', 
                        msg_type=CompressedImage),
                    'frontcam_raw':SubscribeArg(topic='/frontcam/camera/image_color', 
                        msg_type=Image),
                    'bottomcam_compressed':SubscribeArg(topic='/bottomcam/camera/image_color/compressed',
                        msg_type=CompressedImage),
                    'bottomcam_raw':SubscribeArg(topic='/bottomcam/camera/image_color',
                        msg_type=Image)}

    #Details of system-wide publishers used
    system_publishers = {'output':SubscribeArg(topic='/camera_processed/compressed', msg_type=CompressedImage)}

    def __init__(self, name, detector, sub_list=[], pub_list=[]):
        """
        Args:
            name: name of task
            sub_list: list of subscribers used during task
            pub_list: list of publishers used during task
        """
        self.name = name
        self.detector = detector
        self.sub_list = sub_list
        self.pub_list = pub_list

if __name__ == "__main__":
    baseConfig = BaseConfig("Test")
