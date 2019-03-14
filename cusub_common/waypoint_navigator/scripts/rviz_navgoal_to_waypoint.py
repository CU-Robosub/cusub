#!/usr/bin/env python
"""
This module takes in 2D navgoals and sends them to the waypoint navigator to
direct the sub
"""

import rospy

import actionlib

from geometry_msgs.msg import PoseStamped

from waypoint_navigator.msg import waypointAction, waypointGoal

class RVIZNavgoalToWaypoint(object):
    """
    This node takes in 2D navgoals and sends them to the waypoint navigator
    to direct the sub
    """

    waycli = None
    """Waypoint Actionlib Client that sends goals to navigator"""

    def __init__(self):
        pass

    def navgoal_callback(self, navgoal):
        """Gets nav goals and sends them to waypoint navigator"""

        # We just abort any previous goals if we get a new one
        self.waycli.cancel_all_goals()

        # Delay to prevent state machine issues
        rospy.sleep(0.2)

        # Build the waypoint goal
        # Since the 2D Nav Goal has a position and direction we will dirve
        #  with strafe mode.
        wp_goal = waypointGoal()
        wp_goal.goal_pose.header = navgoal.header # directly use header for frame reference and time
        wp_goal.goal_pose.pose = navgoal.pose     # copy pose information in this includes target
                                                  #  yaw
                                                  # TODO depricate target yaw
        wp_goal.movement_mode = waypointGoal.STRAFE_MODE

        # We need a depth because the navgoal from rviz is 2D
        wp_goal.goal_pose.pose.position.z = rospy.get_param("~depth", -2.0)

        # Send new goal
        self.waycli.send_goal(wp_goal)

    def run(self):
        """Initalizes node"""

        rospy.init_node('rviz_navgoal_to_waypoint')

        rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.navgoal_callback)

        self.waycli = actionlib.SimpleActionClient('cusub_common/waypoint', waypointAction)

        rospy.spin()

if __name__ == '__main__':
    RNTW = RVIZNavgoalToWaypoint()
    RNTW.run()
