#!/usr/bin/env python
"""Publishes thrust odometry to stabalize EKF"""

import rospy

from geometry_msgs.msg import TwistWithCovarianceStamped

class ThrustOdom(object):
    """Publishes thrust odometry to stabalize EKF"""

    def run(self):
        """Does publishing of thrust odometry"""

        rospy.init_node('depth_sensor_gazebo', anonymous=True)

        thrust_odom_pub = rospy.Publisher('thrust_odom',
                                          TwistWithCovarianceStamped, queue_size=1)

        thrust_odom = TwistWithCovarianceStamped()
        thrust_odom.header.seq = 0
        thrust_odom.header.frame_id = 'base_link'
        thrust_odom.twist.twist.angular.x = 0
        thrust_odom.twist.twist.angular.y = 0
        thrust_odom.twist.twist.angular.z = 0
        thrust_odom.twist.twist.linear.x = 0
        thrust_odom.twist.twist.linear.y = 0
        thrust_odom.twist.twist.linear.z = 0
        thrust_odom.twist.covariance = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, -1.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, -1.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, -1.0]

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            thrust_odom.header.seq += 1
            thrust_odom.header.stamp = rospy.get_rostime()

            thrust_odom_pub.publish(thrust_odom)

            rate.sleep()

if __name__ == '__main__':
    THRUST_ODOM = ThrustOdom()
    try:
        THRUST_ODOM.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Thurster odometry died!")
