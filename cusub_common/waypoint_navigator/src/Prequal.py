#!/usr/bin/python2

import rospy
import tf
import math

from waypoint_navigator.srv import *
from geometry_msgs.msg import Point

class Prequal:

    current_yaw = 0.0

    def __init__(self):
        pass

    def run(self):

        p1 = Point()
        p1.x =  0.0
        p1.y =  0.0
        p1.z =  2.0

        p2 = Point()
        p2.x =  4.0
        p2.y =  0.0
        p2.z =  2.0

        p3 = Point()
        p3.x = 13.5
        p3.y =  2.5
        p3.z =  2.0

        p4 = Point()
        p4.x = 13.5
        p4.y = -3.5
        p4.z =  2.0

        p5 = Point()
        p5.x = 4.0
        p5.y = 0.0
        p5.z = 2.0


        p6 = Point()
        p6.x =  0.0
        p6.y =  0.0
        p6.z =  2.0

        p7 = Point()
        p7.x =  0.0
        p7.y =  0.0
        p7.z =  0.0

        rospy.sleep(5.0)

        add_waypoint = rospy.ServiceProxy('/addWaypoint', AddWaypoint)
        add_waypoint(p1)
        add_waypoint(p2)
        add_waypoint(p3)
        add_waypoint(p4)
        add_waypoint(p5)
        add_waypoint(p6)
        add_waypoint(p7)

if __name__ == '__main__':
    rospy.init_node('Prequal')
    wn = Prequal()
    try:
        wn.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mother fucker!")
