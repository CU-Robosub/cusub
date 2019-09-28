#!/usr/bin/env python

"""
Responsibilities:
- convert darknet output to elevation + azimuth

- make sure to lookup through a tf tree the pose of the sub at that specific timestamp to synchronize

"""
from detection_tree.dvector import Dvector
from detection_tree.distribution import DvectorDistribution
import rospy

rospy.init_node("detection_tree")
rospy.loginfo("Detection Tree Initializing")