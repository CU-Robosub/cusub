#!/usr/bin/env python
"""
Sometimes Gazebo doesn't publish the ground truth pose (triangular_buoy)
This class publishes odom msgs to mimick gazebo's GT
"""

import rospy
from nav_msgs.msg import Odometry

def main():
    rospy.init_node('odom_faker', anonymous=True)
    pub = rospy.Publisher("/triangular_buoy_1/pose_gt", Odometry, queue_size=10)
    odom = Odometry()
    odom.header.frame_id = "world"
    odom.pose.pose.position.x = rospy.get_param("~xpos")
    odom.pose.pose.position.y = rospy.get_param("~ypos")
    odom.pose.pose.position.z = rospy.get_param("~zpos")

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pub.publish(odom)
        rate.sleep()

if __name__ == "__main__":
    main()