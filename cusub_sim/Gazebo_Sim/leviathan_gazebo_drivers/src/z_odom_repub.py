#!/usr/bin/env python

import rospy
import tf
from nav_msgs.msg import Odometry

new_odom_pub = None

def odom_callback(odom):

    global new_odom_pub

    odom.pose.pose.position.z = -1*odom.pose.pose.position.z;
    br = tf.TransformBroadcaster()
    br.sendTransform((0,0,odom.pose.pose.position.z*2),
                    tf.transformations.quaternion_from_euler(0,0,0),
                    rospy.Time.now(),
                    "odom_z",
                    "base_link")
    new_odom_pub.publish(odom)

def z_odom_repub():

    global new_odom_pub

    rospy.init_node('new_odom_pub', anonymous=True)

    new_odom_pub = rospy.Publisher("/sensor_fusion/odometry/filtered_z_flip", Odometry, queue_size=1)

    pose_sub = rospy.Subscriber("/sensor_fusion/odometry/filtered", Odometry, odom_callback)

    rospy.spin()

if __name__ == '__main__':
    try:
        z_odom_repub()
    except rospy.ROSInterruptException:
        rospy.loginfo("Z Odom Repub Broke")
