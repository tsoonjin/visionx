#!/usr/bin/env python
import abc

class BaseDetector(object):
   """
   Single object detector for OpenCV image format
   """

   __metaclass__ = abc.ABCMeta

   def __init__(self):
       pass

   @abc.abstractmethod
   def detect(self, img):
       """
       Args:
           img: OpenCV compatible image for further processing
       
       Returns:
           output: Result of image processing with details about object of interest
       """
       return

