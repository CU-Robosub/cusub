#!/usr/bin/env python

""" Unit test for triangle buoy
- Test actionlib orbit request
- Test get_approach_pose method
"""

import rospy
import actionlib
from geometry_msgs.msg import Pose
from tasks.triangle_buoy import Slay
from perception_control.msg import OrbitBuoyAction, OrbitBuoyGoal
import matplotlib.pyplot as plt
import numpy as np

if __name__ == '__main__':
    # rospy.init_node('obit_buoy_client')
    # rospy.loginfo("Loading test triangle buoy")
    # client = actionlib.SimpleActionClient('orbit_buoy', OrbitBuoyAction)
    # client.wait_for_server()

    # goal = OrbitBuoyGoal()
    # goal.target_class = "vampire_fathead"
    # goal.buoy_pose = Pose()
    # goal.strafe_setpoint = 2.0
    # goal.number_solo_frames = 10
    # # Fill in the goal here
    # client.send_goal(goal)
    # rospy.sleep(1)
    # # client.cancel_goal()
    # client.wait_for_result(rospy.Duration.from_sec(5.0))
    # print(client.get_result())

    p = Pose()
    p.position.x = 0
    p.position.y = 0
    p2 = Pose()
    p2.position.x = 2
    p2.position.y = 0
    p3 = Slay.get_approach_pose(p, p2, orbit_radius=0.5)
    x = np.array([p.position.x, p2.position.x, p3.position.x])
    y = np.array([p.position.y, p2.position.y, p3.position.y])
    color = np.array([0.0,0.5,1.0])
    plt.scatter(x,y, c=color)
    plt.show()
    ###########
    p2 = Pose()
    p2.position.x = 2
    p2.position.y = 2
    p3 = Slay.get_approach_pose(p, p2, orbit_radius=0.5)
    x = np.array([p.position.x, p2.position.x, p3.position.x])
    y = np.array([p.position.y, p2.position.y, p3.position.y])
    color = np.array([0.0,0.5,1.0])
    plt.scatter(x,y, c=color)
    plt.show()
    ###########
    p2 = Pose()
    p2.position.x = 0
    p2.position.y = 2
    p3 = Slay.get_approach_pose(p, p2, orbit_radius=0.5)
    x = np.array([p.position.x, p2.position.x, p3.position.x])
    y = np.array([p.position.y, p2.position.y, p3.position.y])
    color = np.array([0.0,0.5,1.0])
    plt.scatter(x,y, c=color)
    plt.show()
    ###########
    p2 = Pose()
    p2.position.x = -2
    p2.position.y = 2
    p3 = Slay.get_approach_pose(p, p2, orbit_radius=0.5)
    x = np.array([p.position.x, p2.position.x, p3.position.x])
    y = np.array([p.position.y, p2.position.y, p3.position.y])
    color = np.array([0.0,0.5,1.0])
    plt.scatter(x,y, c=color)
    plt.show()
    ###########
    p2 = Pose()
    p2.position.x = -2
    p2.position.y = 0
    p3 = Slay.get_approach_pose(p, p2, orbit_radius=0.5)
    x = np.array([p.position.x, p2.position.x, p3.position.x])
    y = np.array([p.position.y, p2.position.y, p3.position.y])
    color = np.array([0.0,0.5,1.0])
    plt.scatter(x,y, c=color)
    plt.show()
    ###########
    p2 = Pose()
    p2.position.x = -2
    p2.position.y = -2
    p3 = Slay.get_approach_pose(p, p2, orbit_radius=0.5)
    x = np.array([p.position.x, p2.position.x, p3.position.x])
    y = np.array([p.position.y, p2.position.y, p3.position.y])
    color = np.array([0.0,0.5,1.0])
    plt.scatter(x,y, c=color)
    plt.show()
    ###########
    p2 = Pose()
    p2.position.x = 0
    p2.position.y = -2
    p3 = Slay.get_approach_pose(p, p2, orbit_radius=0.5)
    x = np.array([p.position.x, p2.position.x, p3.position.x])
    y = np.array([p.position.y, p2.position.y, p3.position.y])
    color = np.array([0.0,0.5,1.0])
    plt.scatter(x,y, c=color)
    plt.show()
    ###########
    p2 = Pose()
    p2.position.x = 2
    p2.position.y = -2
    p3 = Slay.get_approach_pose(p, p2, orbit_radius=0.5)
    x = np.array([p.position.x, p2.position.x, p3.position.x])
    y = np.array([p.position.y, p2.position.y, p3.position.y])
    color = np.array([0.0,0.5,1.0])
    plt.scatter(x,y, c=color)
    plt.show()

    print('\n\n')
    print('sub\n----')
    print(p.position)
    print('\ngate\n----')
    print(p2.position)
    print('\ngoal\n----')
    print(p3.position)