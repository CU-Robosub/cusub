#!/usr/bin/env python
"""
This module facilitates republishing of the depth sensor fluid pressure
from gazebo into a depth pose for sensor fusion
"""

import rospy

from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped

class DepthSensor(object):
    """
    This node republishes of the depth sensor fluid pressure
    from gazebo into a depth pose for sensor fusion
    """

    depth_pose = None
    """PoseWithCovarianceStamped : Current depth pose"""

    # Bluerobotics Bar30 Pressure Sensor resolution is 2mm
    resolution_m = 0.002
    """float : Resolution of depths sensor in meters"""

    # TODO needs calibration for gazebo, or we can just go with
    #  the current method of using the depth sensor driver to
    #  calibrate a zero depth
    pressure_offset = 103.2
    """pressure offset to correct pressure to depth at surface"""

    def pressure_callback(self, data):
        """Gets current pressure from gazebo"""

        self.depth_pose.header.stamp = rospy.Time.now()
        self.depth_pose.header.seq += 1

        # 103.2 = 8.748m

        depth = (data.fluid_pressure - self.pressure_offset) / 9.91

        # create stepped depth resolution to match real sensor
        depth = round(depth / self.resolution_m) * self.resolution_m

        # -1 Z up
        self.depth_pose.pose.pose.position.z = -1.0*depth

    def depth_sensor(self):
        """Republishes presure as depth pose"""

        rospy.init_node('depth_sensor_gazebo', anonymous=True)

        rospy.Subscriber('pressure', FluidPressure, self.pressure_callback)

        robot_name = rospy.get_namespace().split('/')[1]

        self.depth_pose = PoseWithCovarianceStamped()
        self.depth_pose.pose.covariance = [0, 0, 0, 0, 0, 0,
                                           0, 0, 0, 0, 0, 0,
                                           0, 0, 0.01, 0, 0, 0,
                                           0, 0, 0, 0, 0, 0,
                                           0, 0, 0, 0, 0, 0,
                                           0, 0, 0, 0, 0, 0]
        self.depth_pose.header.frame_id = robot_name + '/depth_frame'
        self.depth_pose.pose.pose.position.z = 0

        depth_pub = rospy.Publisher('depth', PoseWithCovarianceStamped, queue_size=1)

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():

            depth_pub.publish(self.depth_pose)

            rate.sleep()

if __name__ == '__main__':
    DEPTH_SENSOR = DepthSensor()
    try:
        DEPTH_SENSOR.depth_sensor()
    except rospy.ROSInterruptException:
        rospy.loginfo("Depth sensor died!")
