#!/usr/bin/env python

import rospy

from geometry_msgs.msg import TwistWithCovarianceStamped

dvl_pub = None

def twist_callback(data):

    global dvl_pub
    #data.twist.twist.linear.y = -1*data.twist.twist.linear.y
    data.twist.twist.linear.z = -1*data.twist.twist.linear.z
    dvl_pub.publish(data)

def dvl():

    global dvl_pub

    rospy.init_node('dvl_gazebo', anonymous=True)
    rospy.Subscriber("/leviathan/dvl_twist", TwistWithCovarianceStamped, twist_callback)
    dvl_pub = rospy.Publisher('/drivers/dvl/vel', TwistWithCovarianceStamped, queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    try:
        dvl()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mother fucker!")
