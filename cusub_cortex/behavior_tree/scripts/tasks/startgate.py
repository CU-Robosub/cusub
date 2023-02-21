#!/usr/bin/env python

from detection_listener.listener import DetectionListener

from general import *

class Find_Startgate(Node):

    def __init__(self, name):
        self.name = name
    
    def activate(self):
        print(self.name)
        return Status.SUCCESS