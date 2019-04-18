#!/usr/bin/env python
"""
Naive visual servoing
"""

from tasks.task import Objective, Task
import smach

import rospy

from std_msgs.msg import Float64
from topic_tools.srv import MuxSelect
from darknet_ros_msgs.msg import BoundingBoxes

class NaiveVisualServoTask(Task):

    outcomes = ['task_success','task_aborted']

    def __init__(self, prior):

        super(NaiveVisualServoTask, self).__init__(self.outcomes) # become a state machine first
        self.initObjectives(prior, None)
        self.linkObjectives()

    def initObjectives(self, prior, searchAlg):
        self.servo = NaiveVisualServo(prior)

    def initMapperSubs(self):
        """
        Just going to the prior so no need to act on any pose estimates
        """
        pass

    def linkObjectives(self):
        with self:
            smach.StateMachine.add('Servo', self.servo, transitions={'aborted':'task_aborted', 'success':'task_success'})

class NaiveVisualServo(Objective):
    """
    Visual Servo a bounding box with bangbang control.
    """

    outcomes = ['success', 'aborted']

    target_dice = None

    def boxes_received(self, msg):
        """
        Get the position of the dice
        """

        for box in msg.bounding_boxes: # loop through all of the boxes

            probability = 0.0
            the_one_true_box = None

            # Find most probable dice 6 bounding box
            if box.Class == self.target_dice:
                if box.probability > probability:
                    probability = box.probability
                    the_one_true_box = box

            if the_one_true_box is not None:

                x_center = Float64()
                x_center.data = (the_one_true_box.xmin + the_one_true_box.xmax) / 2.0
                self.dice_state_pub.publish(x_center)

                x_target = Float64()
                x_target.data = 376.0
                self.dice_setpoint_pub.publish(x_target)

    def __init__(self, prior):

        self.prior = prior

        self.target_dice = 'dice5'

        rospy.loginfo("--visual-servoing-init")

        self.robotname = rospy.get_param('~robotname')

        rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes,
                         self.boxes_received, queue_size=1, buff_size=10000000)

        self.dice_state_pub = rospy.Publisher('cusub_common/motor_controllers/cv/yaw/state',
                                              Float64, queue_size=1)
        self.dice_setpoint_pub = rospy.Publisher('cusub_common/motor_controllers/cv/yaw/setpoint',
                                                 Float64, queue_size=1)

        self.yaw_select = rospy.ServiceProxy('cusub_common/motor_controllers/yaw_mux/select',
                                              MuxSelect)

        super(NaiveVisualServo, self).__init__(self.outcomes, "NaiveVisualServo")

    def execute(self, userdata):
        """
        Do visual servoing
        """

        rospy.loginfo("--visual-servoing")
        self.clear_abort()

        # switch to visual servoing
        self.yaw_select("/" + self.robotname +
                       "/cusub_common/motor_controllers/cv/yaw/control_effort")

        # spin till we finish
        rospy.sleep(30)

        # back to point n shoot control
        self.yaw_select("/" + self.robotname +
                       "/cusub_common/motor_controllers/pid/yaw/control_effort")

        return "success"
