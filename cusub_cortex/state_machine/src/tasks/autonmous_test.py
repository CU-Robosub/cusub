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
import math
from geometry_msgs.msg import Pose, Point, Quaternion
from nav_msgs.msg import Odometry
import actionlib
from waypoint_navigator.msg import waypointAction, waypointGoal
from std_msgs.msg import Float64
from tasks.task import Task, Objective

YAW_MODE = 1
STRAFE_MODE = 2
BACKUP_MODE = 3
POSE_REACHED_THRESHOLD = 0.3

class Prequal:

    outcomes=['done']

    def __init__(self, task_name):
        name = task_name + "/PreQual"
        rospy.Subscriber('cusub_common/odometry/filtered', Odometry, self.sub_pose_cb)
        self.wayClient = actionlib.SimpleActionClient('cusub_common/waypoint', waypointAction)

        # initialize the pose
        pose = Pose()
        pose.position.x = 0
        pose.position.y = 0
        pose.position.z = 0
        self.cur_pose = pose

    def sub_pose_cb(self, msg):
        self.cur_pose = msg.pose.pose # store the pose part of the odom msg

    def go_to_pose(self, target_pose, replan_enabled=True, move_mode="yaw"):
        """ @brief traverses to the target_pose given, blocks until reached

        Combination of go_to_pose_non_blocking() and block_on_reaching_pose()

        Call like (if replanning is possible, surround w/ while loop):
        if self.go_to_pose(target_pose, userdata.timeout_obj):
            if userdata.timeout_obj.timed_out:
                userdata.outcome = "timedout"
                return "done"
            else: # Replan has been requested loop again
                pass
        else: # Pose reached successfully!
            pass or break # from while, in the case of replan

        Parameters
        ----------
        target_pose : Pose
             The pose for the sub to navigate to
             Passed by copy instead of reference (like self.target_pose) b/c replanning now allowed
        timeout_obj : almost certainly this is userdata.timeout_obj
        replan_enabled : bool
            True for allowing replans to interrupt the mission
            False for preventing replans from happening
        move_mode : str
             "yaw" : the waypoint navigtator will use yaw mode to navigate to the target pose
             "strafe" : use strafe-drive mode to the target pose
             "backup" : turn 180 deg away from object, backup to target point, turn to target orientation

        Returns
        -------
        bool : 
            0 success, waypoint reached
            1 timedout or replan_requested, user must check the value of 
                timeout_obj.timed_out when this result is returned to determine cause of failure
        """
        self.go_to_pose_non_blocking(target_pose, move_mode)
        return self.block_on_reaching_pose(target_pose, replan_enabled)

    def block_on_reaching_pose(self, target_pose, replan_enabled=True):
        """
        @brief blocks on the already sent waypoint goal

        Parameters
        ----------
        target_pose : Pose
             The pose for the sub to navigate to
        timeout_obj : almost certainly this is userdata.timeout_obj

        Returns
        -------
        bool : 
            0 success, waypoint reached
            1 timedout or replan_requested, user must check the value of 
                timeout_obj.timed_out when this result is returned to determine cause of failure
        """
        while  not rospy.is_shutdown():
            res = self.wayClient.get_result()
            if self.check_reached_pose(target_pose):
                self.cancel_way_client_goal()
                return False
            rospy.sleep(0.25)
        return False

    def check_reached_pose(self, target_pose):
        return self.get_distance(self.cur_pose.position, target_pose.position) < POSE_REACHED_THRESHOLD

    def get_distance(self, point1, point2):
        """ Get distance between 2 points """
        dx = point2.x - point1.x
        dy = point2.y - point1.y
        dz = point2.z - point1.z
        return math.sqrt(dx**2 + dy**2 + dz**2)

    def go_to_pose_non_blocking(self, target_pose, move_mode="yaw"):
        """
        @brief sends a pose to waypoint navigator and returns
        """
        wpGoal = waypointGoal()
        wpGoal.goal_pose.pose.position = target_pose.position
        wpGoal.goal_pose.pose.orientation = target_pose.orientation
        wpGoal.goal_pose.header.frame_id = 'triton/description/odom'

        if move_mode == "backup":
            wpGoal.movement_mode = BACKUP_MODE
        elif move_mode == "strafe":
            wpGoal.movement_mode = STRAFE_MODE
        else:
            wpGoal.movement_mode = YAW_MODE

        self.wayClient.cancel_all_goals()
        rospy.sleep(0.2)
        self.wayClient.send_goal(wpGoal)
        rospy.loginfo("---goal sent to waypointNav")

    def execute(self):

        point1 = Point(3,0,-1)
        quaternion1 = Quaternion(2,2,2,2)
        pose1 = Pose()
        pose1.position = point1
        pose1.orientation = quaternion1

        # self.toggle_waypoint_control(False)
        self.go_to_pose(pose1)
        print("reach 1")

        point2 = Point(3,3,-1)
        quaternion2 = Quaternion(2,2,2,2)
        pose2 = Pose()
        pose2.position = point2
        pose2.orientation = quaternion2

        self.go_to_pose(pose2)
        print("reach 2")

        point3 = Point(0,3,-1)
        quaternion3 = Quaternion(2,2,2,2)
        pose3 = Pose()
        pose3.position = point3
        pose3.orientation = quaternion3

        self.go_to_pose(pose3)
        print("reach 3")

        point4 = Point(0,0,0)
        quaternion4 = Quaternion(2,2,2,2)
        pose4 = Pose()
        pose4.position = point4
        pose4.orientation = quaternion4

        self.go_to_pose(pose4)
        print("reach 4")


        userdata.outcome = "success"
        return "done"

if __name__ == "__main__":
	rospy.init_node("auto_test")
	p = Prequal("auto_test")
	p.execute()

	rospy.spin()
