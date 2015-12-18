#!/usr/bin/env python
from collections import namedtuple

import rospy
from geometry_msgs.msg import Vector3Stamped
from sensor_msgs.msg import CompressedImage, Image

from visionx.msg import *
from visionx.srv import *


class BaseConfig(object):
    """
    Basic configuration that stores details pertaining to ROS related parameters and the task
    """

    #Topic and message type pair tuple
    Message = namedtuple('Message', 'topic type')

    #Details of system-wide msgs
    sys_msgs = {'heading':Message(topic='/navigation/RPY', type=Vector3Stamped), 
                     'depth':Message(topic='/depth', type=depth),
                     'frontcam_compressed':Message(topic='/frontcam/camera/image_color/compressed', 
                         type=CompressedImage),
                     'frontcam_raw':Message(topic='/frontcam/camera/image_color', 
                         type=Image),
                     'bottomcam_compressed':Message(topic='/bottomcam/camera/image_color/compressed', 
                         type=CompressedImage),
                     'bottomcam_raw':Message(topic='/bottomcam/camera/image_color', 
                         type=Image),
                     'vision_output':Message(topic='/camera_processed/compressed',
                         type=CompressedImage),
                     'navigation_server':Message(topic='LocomotionServer', type=ControllerAction), 
                     'pid':Message(topic='/set_controller_srv', type=set_controller)}

    def __init__(self, name, detector):
        """
        Args:
            name: name of task
        """
        self.name = name
        self.detector = detector
        self.publishers = self.generate_publishers(detector.processed)

    def generate_publishers(self, processed):
        topics = [k for k in processed.keys()].extend(('output', 'debug'))
        publishers = {k:rospy.Publisher('/{}/{}/compressed'.format(self.name, k), CompressedImage)
                        for k in topics}

if __name__ == "__main__":
    baseConfig = BaseConfig("lane", None)
