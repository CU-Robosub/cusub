#!/usr/bin/env python
"""
Naive visual servoing
"""

from tasks.task import Objective

import rospy

from std_msgs.msg import Float64
from topic_tools.srv import MuxSelect
from darknet_ros_msgs.msg import BoundingBoxes

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
                self.diceStatePub.publish(x_center)

                x_target = Float64()
                x_target.data = 376.0
                self.diceSetpointPub.publish(x_target)

                y_center = Float64()
                y_center.data = (the_one_true_box.ymin + the_one_true_box.ymax) / 2.0
                self.diceDepthStatePub.publish(y_center)

                y_target = Float64()
                y_target.data = 300.0
                self.diceDepthSetpointPub.publish(y_target)

    def __init__(self, prior):

        self.prior = prior

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

    def charge_dice(self):
        """
        Switch to bangbang control and back
        """

        # switch to visual servoing
        self.yawSelect("/" + self.robotname +
                       "/cusub_common/motor_controllers/cv/yaw/control_effort")

        # spin till we finish

        # back to point n shoot control
        self.yawSelect("/" + self.robotname +
                       "/cusub_common/motor_controllers/pid/yaw/control_effort")

    def execute(self, userdata):
        """
        Do visual servoing
        """

        rospy.loginfo("--visual-servoing")
        self.clear_abort()

        #

        return "success"
