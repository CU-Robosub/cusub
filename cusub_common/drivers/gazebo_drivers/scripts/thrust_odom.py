#!/usr/bin/env python
"""Publishes thrust odometry to stabalize EKF"""

import rospy

from std_msgs.msg import Float64
from geometry_msgs.msg import TwistWithCovarianceStamped

class ThrustOdom(object):
    """Publishes thrust odometry to stabalize EKF"""

    drive_effort = 0.0
    strafe_effort = 0.0

    def drive_effort_callback(self, effort):
        """Drive effort to estimate x velocity"""
        self.drive_effort = effort.data

    def strafe_effort_callback(self, effort):
        """Strafe effort to estimate y velocity"""
        self.strafe_effort = effort.data

    def run(self):
        """Does publishing of thrust odometry"""

        rospy.init_node('thrust_odom_gazebo', anonymous=True)

        thrust_odom_pub = rospy.Publisher('cusub_common/thrust_odom',
                                          TwistWithCovarianceStamped, queue_size=1)

        rospy.Subscriber('cusub_common/motor_controllers/mux/drive/control_effort',
                         Float64, self.drive_effort_callback)
        rospy.Subscriber('cusub_common/motor_controllers/mux/strafe/control_effort',
                         Float64, self.strafe_effort_callback)

        thrust_odom = TwistWithCovarianceStamped()
        thrust_odom.header.seq = 0
        thrust_odom.header.frame_id = rospy.get_namespace()[1:] + 'description/base_link'
        thrust_odom.twist.twist.angular.x = 0
        thrust_odom.twist.twist.angular.y = 0
        thrust_odom.twist.twist.angular.z = 0
        thrust_odom.twist.twist.linear.x = 0
        thrust_odom.twist.twist.linear.y = 0
        thrust_odom.twist.twist.linear.z = 0
        thrust_odom.twist.covariance = [0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, -1.0, 0.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, -1.0, 0.0,
                                        0.0, 0.0, 0.0, 0.0, 0.0, -1.0]

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            thrust_odom.header.seq += 1
            thrust_odom.header.stamp = rospy.get_rostime()

            thrust_odom.twist.twist.linear.x = self.drive_effort / 200.0
            thrust_odom.twist.twist.linear.y = self.strafe_effort / -200.0

            thrust_odom_pub.publish(thrust_odom)

            rate.sleep()

if __name__ == '__main__':
    THRUST_ODOM = ThrustOdom()
    try:
        THRUST_ODOM.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Thurster odometry died!")
