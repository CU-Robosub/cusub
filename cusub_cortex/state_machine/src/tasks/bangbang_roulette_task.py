#!/usr/bin/env python

"""
Wrecks dem Roulette
"""

from tasks.task import Task, Objective
from tasks.search import Search
import rospy
import smach

import tf

from std_msgs.msg import Float64
from geometry_msgs.msg import Quaternion
from topic_tools.srv import MuxSelect
from sensor_msgs.msg import Imu
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

from actuator.srv import ActivateActuator

class BangBangRouletteTask(Task):

    outcomes = ['task_success','task_aborted']

    def __init__(self, prior, searchAlg):

        super(BangBangRouletteTask, self).__init__(self.outcomes) # become a state machine first
        self.initObjectives(prior, searchAlg)
        self.linkObjectives()

    def initObjectives(self, prior, searchAlg):
        #self.search = Search(searchAlg, prior)
        self.attack = Attack()

    def initMapperSubs(self):
        """
        Just going to the prior so no need to act on any pose estimates
        """
        pass

    def linkObjectives(self):
        with self:
            #smach.StateMachine.add('Search', self.search, transitions={'aborted':'task_aborted', 'success':'Attack'})

            smach.StateMachine.add('Attack', self.attack, transitions={'success':'task_success', 'aborted':'Attack'})

class Attack(Objective):
    """
    Tell the sub to wreck them Roulette
    """

    outcomes=['success','aborted']

    target_Roulette = None

    def boxes_received(self,msg):
        """
        Get the position of the Roulette
        """

        for box in msg.bounding_boxes: # loop through all of the boxes

            probability = 0.0
            theOneTrueBox = None

            # Find most probable roulette bounding box
            if box.Class == "roulette":
                if(box.probability > probability):
                    probability = box.probability
                    theOneTrueBox = box

            if theOneTrueBox is not None:

                width = theOneTrueBox.xmax - theOneTrueBox.xmin
                print(width)
                if width > 900:
                    self.decend = False

                # W 1280
                # H 960

                xCenter = Float64()
                xCenter.data = (theOneTrueBox.xmin + theOneTrueBox.xmax) / 2.0
                self.cv_drive_state_pub.publish(xCenter)

                xTarget = Float64()
                xTarget.data = 640
                self.cv_strafe_setpoint_pub.publish(xTarget)

                yCenter = Float64()
                yCenter.data = 960 - ((theOneTrueBox.ymin + theOneTrueBox.ymax) / 2.0)
                self.cv_strafe_state_pub.publish(yCenter)

                yTarget = Float64()
                yTarget.data = 480
                self.cv_drive_setpoint_pub.publish(yTarget)

    def depth_state_callback(self, data):
        self.current_depth = data.data

    def __init__(self):

        rospy.loginfo("---Attack objective initializing")

        self.robotname = rospy.get_param('~robotname')

        # Your bounding boxes, give them to me...  NOW!
        #TODO namesapcing
        self.darknetSub = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.boxes_received, queue_size=1, buff_size=10000000)

        self.cv_drive_state_pub = rospy.Publisher('cusub_common/motor_controllers/cv/drive/state', Float64, queue_size=1)
        self.cv_drive_setpoint_pub = rospy.Publisher('cusub_common/motor_controllers/cv/drive/setpoint', Float64, queue_size=1)

        self.cv_strafe_state_pub = rospy.Publisher('cusub_common/motor_controllers/cv/strafe/state', Float64, queue_size=1)
        self.cv_strafe_setpoint_pub = rospy.Publisher('cusub_common/motor_controllers/cv/strafe/setpoint', Float64, queue_size=1)

        self.drive_select = rospy.ServiceProxy('cusub_common/motor_controllers/drive_mux/select', MuxSelect)
        self.strafe_select = rospy.ServiceProxy('cusub_common/motor_controllers/strafe_mux/select', MuxSelect)

        self.depth_pub = rospy.Publisher('cusub_common/motor_controllers/pid/depth/setpoint', Float64, queue_size=1)
        rospy.Subscriber("cusub_common/motor_controllers/pid/depth/state", Float64, self.depth_state_callback)

        super(Attack, self).__init__(self.outcomes, "Attack")

    def execute(self, userdata):

        rospy.loginfo("Executing Attack")
        self.clear_abort()

        self.drive_select("/" + self.robotname + "/cusub_common/motor_controllers/cv/drive/control_effort")
        self.strafe_select("/" + self.robotname + "/cusub_common/motor_controllers/cv/strafe/control_effort")

        rospy.loginfo("--lockon")
        rospy.sleep(5.0)

        # Decend
        self.decend = True
        depth = Float64()
        for i in xrange(160):

            rospy.loginfo(self.current_depth)

            depth.data = self.current_depth - 0.025
            self.depth_pub.publish(depth)
            if not self.decend or self.current_depth <= -3.75:
                depth.data = self.current_depth
                self.depth_pub.publish(depth)
                rospy.loginfo("--depth reached")
                break
            rospy.sleep(0.25)

        rospy.loginfo("--drop")

        # activate_actuator = rospy.ServiceProxy('cusub_common/activateActuator', ActivateActuator)
        # activate_actuator(1, 100) # Activate actuator 1 for 100 seconds

        self.drive_select("/" + self.robotname + "/cusub_common/motor_controllers/pid/drive/control_effort")
        self.strafe_select("/" + self.robotname + "/cusub_common/motor_controllers/pid/strafe/control_effort")

        return "success"
