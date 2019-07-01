#!/usr/bin/env python

"""
Test actionlib request from task code
"""

import rospy
import actionlib
from geometry_msgs.msg import Pose

from perception_control.msg import OrbitBuoyAction, OrbitBuoyGoal

if __name__ == '__main__':
    rospy.init_node('obit_buoy_client')
    rospy.loginfo("Loading test triangle buoy")
    client = actionlib.SimpleActionClient('orbit_buoy', OrbitBuoyAction)
    client.wait_for_server()

    goal = OrbitBuoyGoal()
    goal.target_class = "vampire_fathead"
    goal.buoy_pose = Pose()
    goal.strafe_setpoint = 2.0
    goal.number_solo_frames = 10
    # Fill in the goal here
    client.send_goal(goal)
    rospy.sleep(1)
    # client.cancel_goal()
    client.wait_for_result(rospy.Duration.from_sec(5.0))
    print(client.get_result())