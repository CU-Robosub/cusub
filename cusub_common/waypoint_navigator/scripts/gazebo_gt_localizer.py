#!/usr/bin/env python
"""
This module takes in ground truth from gazebo and feeds it to the mapper EKF
to localize the robot
"""

import rospy

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry

class GazeboGTLocalizer(object):
    """
    This node takes in ground truth from gazebo and feeds it to the mapper EKF
    to localize the robot
    """

    gt_pose = None
    """The ground truth pose to send to mapper EKF"""

    map_pose_pub = None
    """The mapper EKF pose topic publisher"""

    def __init__(self):
        pass

    def ground_truth_callback(self, odom):
        """Takes the ground truth and feeds to to mapper EKF"""

        self.gt_pose.pose.pose.position = odom.pose.pose.position
        self.gt_pose.pose.pose.orientation = odom.pose.pose.orientation

        self.gt_pose.header = odom.header

        self.map_pose_pub.publish(self.gt_pose)

    def run(self):
        """Initalizes node"""

        rospy.init_node('gazebo_gt_localizer', anonymous=True)

        rospy.loginfo("Gazebo GT Localizer started")

        self.gt_pose = PoseWithCovarianceStamped()
        self.gt_pose.pose.covariance = [0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0,
                                        0, 0, 0.01, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0,
                                        0, 0, 0, 0, 0, 0]


        self.map_pose_pub = rospy.Publisher("cusub_common/map_pose",
                                            PoseWithCovarianceStamped, queue_size=1)

        rospy.Subscriber('description/pose_gt', Odometry, self.ground_truth_callback)

        rospy.spin()

if __name__ == '__main__':
    GGTL = GazeboGTLocalizer()
    GGTL.run()
