#!/usr/bin/python2

import rospy
import tf
import math

from waypoint_navigator.srv import *
from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

class MagnetometerCalibrator:

    current_yaw = 0.0

    def __init__(self):
        pass

    def calibrate(self, req):

        cur_dec = rospy.get_param('/declination', 0.0)
        requested_yaw = req.heading.data
        declination = math.degrees(self.imu_yaw + math.radians(cur_dec) - requested_yaw)
        rospy.set_param('/declination', declination)
        rospy.loginfo("Setting declination to: %f" % declination)
        return []

    def odometryCallback(self, odom):

        position = odom.pose.pose.position
        orientation = odom.pose.pose.orientation

        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y,orientation.z,orientation.w])

        #rospy.loginfo("SF Yaw: %f" % yaw);

        self.current_yaw = yaw

    def imuCallback(self, imu):

        orientation = imu.orientation
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y,orientation.z,orientation.w])

        #rospy.loginfo("IMU Yaw: %f" % yaw);

        self.imu_yaw = yaw

    def run(self):

        # get current sub orientation to zero the magnetometer
        self.pose_sub = rospy.Subscriber("/sensor_fusion/odometry/filtered", Odometry, self.odometryCallback)
        self.imu_data = rospy.Subscriber("/imu/data", Imu, self.imuCallback)

        # service to activate magnetometer calibration
        s = rospy.Service('calibrateMagnetometer', CalibrateMagnetometer, self.calibrate)

        rospy.spin()


if __name__ == '__main__':
    rospy.init_node('MagnetometerCalibrator')
    wn = MagnetometerCalibrator()
    try:
        wn.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mother fucker!")
