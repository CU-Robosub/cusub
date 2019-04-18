#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu



class IMURepub(object):

    def __init__(self):
        rospy.init_node('imu_repub', anonymous=True)

        self.imu_pub = rospy.Publisher("cusub_common/imu", Imu, queue_size=1)
        self.imu_sub = rospy.Subscriber("description/imu", Imu, self.imu_callback)

    def imu_callback(self, imu):

        self.imu_pub.publish(imu)


if __name__ == '__main__':
    try:
        repub = IMURepub()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Imu Repub broke")
