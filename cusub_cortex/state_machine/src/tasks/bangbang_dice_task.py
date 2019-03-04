#!/usr/bin/env python

"""
Wrecks dem dice
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

class BangBangDiceTask(Task):

    outcomes = ['task_success','task_aborted']

    def __init__(self, prior, searchAlg):

        super(BangBangDiceTask, self).__init__(self.outcomes) # become a state machine first
        self.initObjectives(prior, searchAlg)
        self.linkObjectives()

    def initObjectives(self, prior, searchAlg):
        self.search = Search(searchAlg, prior)
        self.attack = Attack(prior)

    def initMapperSubs(self):
        """
        Just going to the prior so no need to act on any pose estimates
        """
        pass

    def linkObjectives(self):
        with self:
            smach.StateMachine.add('Search', self.search, transitions={'aborted':'task_aborted', 'success':'Attack'})

            smach.StateMachine.add('Attack', self.attack, transitions={'success':'task_success', 'aborted':'Attack'})

class Attack(Objective):
    """
    Tell the sub to wreck them dice
    """

    outcomes=['success','aborted']

    target_dice = None

    def boxes_received(self,msg):
        """
        Get the position of the dice
        """

        for box in msg.bounding_boxes: # loop through all of the boxes

            probability = 0.0
            theOneTrueBox = None

            # Find most probable dice 6 bounding box
            if box.Class == self.target_dice:
                if(box.probability > probability):
                    probability = box.probability
                    theOneTrueBox = box

            if theOneTrueBox is not None:

                xCenter = Float64()
                xCenter.data = (theOneTrueBox.xmin + theOneTrueBox.xmax) / 2.0
                self.diceStatePub.publish(xCenter)

                xTarget = Float64()
                xTarget.data = 376.0
                self.diceSetpointPub.publish(xTarget)

                yCenter = Float64()
                yCenter.data = (theOneTrueBox.ymin + theOneTrueBox.ymax) / 2.0
                self.diceDepthStatePub.publish(yCenter)

                yTarget = Float64()
                yTarget.data = 300.0
                self.diceDepthSetpointPub.publish(yTarget)

    def driveStateCallback(self, data):
        self.currentDrive = data.data

    def yawStateCallback(self, data):
        self.currentYaw = data.data

    def depthStateCallback(self, data):
        self.currentDepth = data.data

    def imuCallback(self, imu):
        if imu.linear_acceleration.x < -0.15:
            self.spike = True

    def __init__(self, prior):

        self.prior = prior

        rospy.loginfo("---Attack objective initializing")

        # Your bounding boxes, give them to me...  NOW!
        self.darknetSub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.boxes_received, queue_size=1, buff_size=10000000)

        self.diceStatePub = rospy.Publisher('/leviathan/local_control/cv/yaw/state', Float64, queue_size=1)
        self.diceSetpointPub = rospy.Publisher('/leviathan/local_control/cv/yaw/setpoint', Float64, queue_size=1)

        self.diceDepthStatePub = rospy.Publisher('/leviathan/local_control/cv/depth/state', Float64, queue_size=1)
        self.diceDepthSetpointPub = rospy.Publisher('/leviathan/local_control/cv/depth/setpoint', Float64, queue_size=1)

        self.driveStateSub = rospy.Subscriber("/leviathan/local_control/pid/drive/state", Float64, self.driveStateCallback)
        self.drivePub = rospy.Publisher('/leviathan/local_control/pid/drive/setpoint', Float64, queue_size=1)

        self.yawStateSub = rospy.Subscriber("/leviathan/local_control/pid/yaw/state", Float64, self.yawStateCallback)

        self.depthStateSub = rospy.Subscriber("/leviathan/local_control/pid/depth/state", Float64, self.depthStateCallback)

        self.imuSub = rospy.Subscriber("/leviathan/imu", Imu, self.imuCallback)

        self.yawSelect = rospy.ServiceProxy('/leviathan_yaw_mux/select', MuxSelect)
        self.depthSelect = rospy.ServiceProxy('/leviathan_depth_mux/select', MuxSelect)

        super(Attack, self).__init__(self.outcomes, "Attack")

    def charge_dice(self):

        # switch to visual servoing
        self.yawSelect("/leviathan/local_control/cv/yaw/control_effort")
        self.depthSelect("/leviathan/local_control/cv/depth/control_effort")

        # wait a bit to lock on
        rospy.loginfo("--locking on")
        rospy.sleep(5.0)

        self.target_yaw = self.currentYaw
        self.target_depth = self.currentDepth
        rospy.loginfo("depth " + str(self.target_depth))

        # CHARGE
        rospy.loginfo("--get wrecked")

        drive = Float64()
        self.spike = False
        for i in xrange(200):
            drive.data = self.currentDrive + 1.7
            self.drivePub.publish(drive)
            if self.spike:
                drive.data = self.currentDrive
                self.drivePub.publish(drive)
                rospy.loginfo("--spiked")
                break
            rospy.sleep(0.25)

        # back to point n shoot control
        self.yawSelect("/leviathan/local_control/pid/yaw/control_effort")
        self.depthSelect("/leviathan/local_control/pid/depth/control_effort")

    def execute(self, userdata):

        rospy.loginfo("Executing Attack")
        self.clear_abort()

        self.target_dice = "dice6"
        self.charge_dice()

        rospy.loginfo("--wrecked")

        rospy.loginfo("--to prior")

        target_pose = self.prior
        oldz = self.prior.position.z
        quaternion = tf.transformations.quaternion_from_euler(0, 0, self.target_yaw)
        target_pose.position.z = self.target_depth
        target_pose.orientation.x = quaternion[0]
        target_pose.orientation.y = quaternion[1]
        target_pose.orientation.z = quaternion[2]
        target_pose.orientation.w = quaternion[3]
        self.goToPose(target_pose, useYaw=False)

        self.prior.position.z = oldz
        self.goToPose(target_pose, useYaw=False)

        self.target_dice = "dice5"
        self.charge_dice()

        rospy.loginfo("--wrecked")

        target_pose.position.z = self.target_depth
        self.goToPose(target_pose, useYaw=False)

        self.prior.position.z = oldz
        self.goToPose(target_pose, useYaw=False)

        return "success"