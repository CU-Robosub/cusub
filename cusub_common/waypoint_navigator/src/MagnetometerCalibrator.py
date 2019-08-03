#!/usr/bin/python2

import rospy
import tf
import math
import yaml
import io

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

        declination_data = {'declination': declination}
        with io.open('declination.yaml', 'w', encoding='utf8') as declination_file:
            yaml.dump(declination_data, declination_file, default_flow_style=False, allow_unicode=True)

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

        declination_data = None
        try:
            with open("declination.yaml", "r") as stream:
                declination_data = yaml.safe_load(stream)
        except:
            rospy.logwarn("No declination saved!")

        if declination_data is not None:
            print declination_data

        # get current sub orientation to zero the magnetometer
        self.pose_sub = rospy.Subscriber("odometry/filtered", Odometry, self.odometryCallback)
        self.imu_data = rospy.Subscriber("imu", Imu, self.imuCallback)

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
