#! /usr/bin/env python

import rospy
import actionlib

from WaypointNavigator import WaypointNavigator

from waypoint_navigator.msg import waypointAction, waypointResult

class WaypointServer(WaypointNavigator):

    def __init__(self):
        super(WaypointServer, self).__init__()

    def advanceWaypoint(self):
        rospy.loginfo("Waypoint Reached!")
        self.waypoint = None

    # called everytime a new goal is sent
    def execute(self, goal):
#        rospy.loginfo("Preempt Status: " + str(self.server.__dict__))
        self.waypoint = goal.goal_pose.pose.position
        self.movement_mode = goal.movement_mode
        self.waypoint_yaw = goal.target_yaw
        rospy.loginfo(self.waypoint)
        while(self.waypoint is not None):
            if self.server.is_preempt_requested():

                rospy.loginfo("Preempt Received!")

                result = waypointResult()
                result.complete = False
                self.server.set_succeeded(result)

                return

            else:
                rospy.sleep(0.1)

        result = waypointResult()
        result.complete = True
        self.server.set_succeeded(result)

    def run(self):

        self.server = actionlib.SimpleActionServer('waypoint', waypointAction, self.execute, False)
        self.server.start()

        super(WaypointServer, self).run()

if __name__ == '__main__':
    rospy.init_node('WaypointActionServer')
    ws = WaypointServer()
    try:
        ws.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("WaypointActionServer Crashed")
