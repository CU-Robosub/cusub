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

    name = "bangbang_dice"
    outcomes = ['task_success','task_aborted']

    def __init__(self):

        super(BangBangDiceTask, self).__init__(self.outcomes) # become a state machine first
        self.initObjectives(self.getPrior(), rospy.get_param("tasks/" + self.name + "/search_alg"))
        self.linkObjectives()

    def initObjectives(self, prior, searchAlg):
        self.search = Search(searchAlg, prior)
        self.attack = Attack(prior)

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

        self.robotname = rospy.get_param('~robotname')

        # Your bounding boxes, give them to me...  NOW!
        # TODO namespacing
        self.darknetSub = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes, self.boxes_received, queue_size=1, buff_size=10000000)

        # Okay this is what we'll use, interesting what's the cv?
        self.diceStatePub = rospy.Publisher('cusub_common/motor_controllers/cv/yaw/state', Float64, queue_size=1)
        self.diceSetpointPub = rospy.Publisher('cusub_common/motor_controllers/cv/yaw/setpoint', Float64, queue_size=1)

        # Depth and drive control those, just hold them in place, huh cv again here?
        self.diceDepthStatePub = rospy.Publisher('cusub_common/motor_controllers/cv/depth/state', Float64, queue_size=1)
        self.diceDepthSetpointPub = rospy.Publisher('cusub_common/motor_controllers/cv/depth/setpoint', Float64, queue_size=1)
        self.driveStateSub = rospy.Subscriber("cusub_common/motor_controllers/pid/drive/state", Float64, self.driveStateCallback)
        self.drivePub = rospy.Publisher('cusub_common/motor_controllers/pid/drive/setpoint', Float64, queue_size=1)
        self.imuSub = rospy.Subscriber("cusub_common/imu", Imu, self.imuCallback)

        self.yawStateSub = rospy.Subscriber("cusub_common/motor_controllers/pid/yaw/state", Float64, self.yawStateCallback)
        self.depthStateSub = rospy.Subscriber("cusub_common/motor_controllers/pid/depth/state", Float64, self.depthStateCallback)

        self.yawSelect = rospy.ServiceProxy('cusub_common/motor_controllers/yaw_mux/select', MuxSelect)
        self.depthSelect = rospy.ServiceProxy('cusub_common/motor_controllers/depth_mux/select', MuxSelect)

        super(Attack, self).__init__(self.outcomes, "Attack")

    def charge_dice(self):

        # switch to visual servoing
        self.yawSelect("/" + self.robotname + "/cusub_common/motor_controllers/cv/yaw/control_effort")
        self.depthSelect("/" + self.robotname + "/cusub_common/motor_controllers/cv/depth/control_effort")

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
        self.yawSelect("/" + self.robotname + "/cusub_common/motor_controllers/pid/yaw/control_effort")
        self.depthSelect("/" + self.robotname + "/cusub_common/motor_controllers/pid/depth/control_effort")

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
