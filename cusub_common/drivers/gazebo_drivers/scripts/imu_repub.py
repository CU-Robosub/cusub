#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
from std_msgs.msg import Float64

from tf.transformations import euler_from_quaternion, quaternion_from_euler

import numpy as np

class IMURepub(object):

    def __init__(self):
        rospy.init_node('imu_repub', anonymous=True)

        self.imu_pub = rospy.Publisher("cusub_common/imu", Imu, queue_size=1)
        self.imu_sub = rospy.Subscriber("description/imu", Imu, self.imu_callback)

        self.off_sub = rospy.Subscriber("mag_offset", Float64, self.offset_callback)

        self.magnetometer_offset = 0.0

    def offset_callback(self, offset):

        # Convert offset from degrees to radians and set it
        self.magnetometer_offset = offset.data * np.pi / 180.0

    def imu_callback(self, imu):

        # Offset magnetometer to simulate incorrect calibration
        orientation_q = imu.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        yaw += self.magnetometer_offset

        quat = quaternion_from_euler(roll, pitch,yaw)
        imu.orientation.x = quat[0]
        imu.orientation.y = quat[1]
        imu.orientation.z = quat[2]
        imu.orientation.w = quat[3]

        self.imu_pub.publish(imu)


if __name__ == '__main__':
    try:
        repub = IMURepub()
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("Imu Repub broke")
