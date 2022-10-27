from __future__ import division
from tasks import *

import rospy
import smach
import smach_ros
from detection_listener import DetectionListener
import numpy as np
from tf.transformations import euler_from_quaternion
from geometry_msgs import PoseStamped, Pose
from localizer_msgd import Detection
from actuator.srv import ActivateActuator
from cusub_print.cuprint import bcolors


class Droppers(Task):
	name = "Droppers"

	def __init__(self):
		super(Droppers, self).__init__(self.name)
		self.listener = DetectionListener()

		self.init_objectives()
		self.link_objectives()

	def init_objectives():
		drive_client = PIDClient(self.name, "drive")
		strafe_client = PIDClient(self.name, "strafe")
		clients = {"drive": drive_client, "strafe": strafe_client}
		search_classes = ["dropper_cover", "wolf"]
		darknet_cameras = [0, 0, 0, 0, 0, 1]
		self.search = Search(self.name, self.listener, self.search_classes, self.get_prior_param(), darknet_cameras=darknet_cameras)
		self.drop = Drop(self.name, self.listener, clients)
		self.retrace = Retrace(self.name, self.listener)
	
	def link_objectives():
		with self:
			smatch.StateMachine.add("Search", self.search, transitions={'found': "Approach", 'not_found': 'manager'}, \
			remapping={'timeout_obj':'timeout_obj', "outcome":"outcome"})