#!/usr/bin/env python

"""
Wrecks dem dice
"""

from tasks.task import Task, Objective
from tasks.search import Search
import rospy
import smach

from std_msgs.msg import Float64
from topic_tools.srv import MuxSelect
from darknet_ros_msgs.msg import BoundingBox, BoundingBoxes

class BangBangDiceTask(Task):

    outcomes = ['task_success','task_aborted']

    def __init__(self, prior, searchAlg):

        super(BangBangDiceTask, self).__init__(self.outcomes) # become a state machine first
        self.initObjectives(prior, searchAlg)
        self.linkObjectives()

    def initObjectives(self, prior, searchAlg):
        self.search = Search(searchAlg, prior)
        self.attack = Attack()

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

    def boxes_received(self,msg):
        """
        Get the position of the dice
        """

        for box in msg.bounding_boxes: # loop through all of the boxes

            probability = 0.0
            theOneTrueBox = None

            # Find most probable dice 6 bounding box
            if box.Class == "dice6":
                if(box.probability > probability):
                    probability = box.probability
                    theOneTrueBox = box

            if theOneTrueBox is not None:

                xCenter = Float64()
                xCenter.data = (theOneTrueBox.xmin + theOneTrueBox.xmax) / 2.0
                self.diceStatePub.publish(xCenter);

                xTarget = Float64()
                xTarget.data = 376.0
                self.diceSetpointPub.publish(xTarget);

    def driveStateCallback(self, data):
        self.currentDrive = data.data

    def __init__(self):

        rospy.loginfo("---Attack objective initializing")

        # Your bounding boxes, give them to me...  NOW!
        self.darknetSub = rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.boxes_received, queue_size=1, buff_size=10000000)

        self.diceStatePub = rospy.Publisher('/leviathan/local_control/cv/yaw/state', Float64, queue_size=1)
        self.diceSetpointPub = rospy.Publisher('/leviathan/local_control/cv/yaw/setpoint', Float64, queue_size=1)

        self.driveStateSub = rospy.Subscriber("/leviathan/local_control/pid/drive/state", Float64, self.driveStateCallback)
        self.drivePub = rospy.Publisher('/leviathan/local_control/pid/drive/setpoint', Float64, queue_size=1)

        self.yawSelect = rospy.ServiceProxy('/yaw_mux/select', MuxSelect)

        super(Attack, self).__init__(self.outcomes, "Attack")

    def execute(self, userdata):
        rospy.loginfo("Executing Attack")
        self.clear_abort()

        # switch to visual servoing
        self.yawSelect("/leviathan/local_control/cv/yaw/control_effort")

        # wait a bit to lock on
        rospy.loginfo("--locking on")
        rospy.sleep(2.0)

        # CHARGE
        rospy.loginfo("--get wrecked")
        drive = Float64()
        drive.data = self.currentDrive + 5.0
        self.drivePub.publish(drive)

        # For now we just charge it and after 5 seconds assume we hit it
        rospy.sleep(5.0)

        # back to point n shoot control
        self.yawSelect("/leviathan/local_control/pid/yaw/control_effort")

        rospy.loginfo("--wrecked")

        return "success"
