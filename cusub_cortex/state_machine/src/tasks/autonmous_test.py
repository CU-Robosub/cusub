#!/usr/bin/env python
from __future__ import division
"""
Startup Task, allows the man on the competition dock time to remove the tether from the vehicle before it starts its autonomous run.
Waits briefly at the surface before diving.
"""
import rospy
import smach
import smach_ros
import rospy
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Float64
from tasks.task import Task, Objective


class Prequal(Objective):

    outcomes=['done']

    def __init__(self, task_name):
        name = task_name + "/PreQual"
        super(Prequal, self).__init__(self.outcomes, name)

    def execute(self, userdata):

        point1 = Point(5,0,-2)
        quaternion1 = Quaternion(2,2,2,2)
        pose1 = Pose()
        pose1.position = point1
        pose1.orientation = quaternion1

        self.toggle_waypoint_control(False)
        self.go_to_pose(pose1,userdata.timeout_obj)
        self.cuprint("reach 1")

        point2 = Point(5,5,-2)
        quaternion2 = Quaternion(2,2,2,2)
        pose2 = Pose()
        pose2.position = point2
        pose2.orientation = quaternion2

        self.go_to_pose(pose2,userdata.timeout_obj)
        self.cuprint("reach 2")

        point3 = Point(0,5,-2)
        quaternion3 = Quaternion(2,2,2,2)
        pose3 = Pose()
        pose3.position = point3
        pose3.orientation = quaternion3

        self.go_to_pose(pose3,userdata.timeout_obj)
        self.cuprint("reach 3")

        point4 = Point(0,0,0)
        quaternion4 = Quaternion(2,2,2,2)
        pose4 = Pose()
        pose4.position = point4
        pose4.orientation = quaternion4

        self.go_to_pose(pose4,userdata.timeout_obj)
        self.cuprint("reach 4")


        userdata.outcome = "success"
        return "done"

if __name__ == "__main__":
	rospy.init_node("auto_test")
	p = Prequal("auto_test")
	p.execute()

	rospy.spin()
