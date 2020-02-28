#! /usr/bin/env python
"""Waypoint Navigation"""

import rospy
import actionlib

import tf
from tf.transformations import euler_from_quaternion

from waypoint_navigator.msg import waypointAction, waypointResult

from WaypointNavigator import WaypointNavigator

class WaypointServer(WaypointNavigator):
    """Waypoint Navigation"""

    listener = None
    """Transform listener"""

    server = None
    """Waypoint action client server"""

    def __init__(self):
        super(WaypointServer, self).__init__()

    def advance_waypoint(self):
        """Called when waypoint reached"""
        self.cuprint("Waypoint Reached!!!")
        self.waypoint = None
        self.do_freeze()

    def get_waypoint_in_odom(self, pose):
        """Transforms waypoint into the odom frame for navigation"""

        odom_frame = rospy.get_namespace().split("/")[1] \
                     + "/description/odom"

        pose.header.stamp = rospy.get_rostime()

        self.listener.waitForTransform(pose.header.frame_id, odom_frame,
                                       rospy.get_rostime(), rospy.Duration(0.5))
        map_pose = self.listener.transformPose(odom_frame, pose)

        orientation_list = [map_pose.pose.orientation.x, \
                            map_pose.pose.orientation.y, \
                            map_pose.pose.orientation.z, \
                            map_pose.pose.orientation.w]
        _, _, yaw = euler_from_quaternion(orientation_list)

        return map_pose.pose.position, yaw

    # called everytime a new goal is sent
    def execute(self, goal):
        """Gets a waypoint goal and navigates to it"""

        # rospy.loginfo("Preempt Status: " + str(self.server.__dict__))

        self.waypoint, self.waypoint_yaw = self.get_waypoint_in_odom(goal.goal_pose)
        self.movement_mode = goal.movement_mode

        self.cuprint(str(self.waypoint))

        while self.waypoint is not None:

            # Periodicaly update the waypoint in the
            #  map frame incase our odom point is
            #  out of date
            self.waypoint, self.waypoint_yaw = self.get_waypoint_in_odom(goal.goal_pose)

            if self.server.is_preempt_requested():

                self.cuprint("preempt received")

                result = waypointResult()
                result.complete = False
                self.server.set_preempted(result)
                self.waypoint = None
                self.do_freeze()

                return

            else:

                rospy.sleep(0.1)

        result = waypointResult()
        result.complete = True
        self.server.set_succeeded(result)

    def run(self):

        self.server = actionlib.SimpleActionServer('waypoint', waypointAction, self.execute, False)
        self.server.start()

        self.listener = tf.TransformListener()

        super(WaypointServer, self).run()

if __name__ == '__main__':
    rospy.init_node('waypoint_action_server')
    WAYPOINT_SERVER = WaypointServer()
    try:
        WAYPOINT_SERVER.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("WaypointActionServer Crashed")
