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

        print "got pressure"
        self.depth_odom_pose.header.stamp = rospy.Time.now()
        self.depth_odom_pose.header.seq += 1

        self.depth_map_pose.header.stamp = rospy.Time.now()
        self.depth_map_pose.header.seq += 1

        # 103.2 = 8.748m

        depth = (data.fluid_pressure - self.pressure_offset) / 9.91

        # create stepped depth resolution to match real sensor
        depth = round(depth / self.resolution_m) * self.resolution_m

        # -1 Z up
        self.depth_odom_pose.pose.pose.position.z = -1.0*depth
        self.depth_map_pose.pose.pose.position.z = -1.0*depth

    def depth_sensor(self):
        """Republishes presure as depth pose"""

        rospy.init_node('depth_sensor_gazebo', anonymous=True)

        rospy.Subscriber('description/pressure', FluidPressure, self.pressure_callback)

        self.depth_odom_pose = PoseWithCovarianceStamped()
        self.depth_odom_pose.pose.covariance = [0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0.01, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0,
                                                0, 0, 0, 0, 0, 0]

        self.depth_odom_pose.header.frame_id = rospy.get_param('~namespace') + '/description/depth_odom_frame'
        self.depth_odom_pose.pose.pose.position.z = 0

        self.depth_map_pose = PoseWithCovarianceStamped()
        self.depth_map_pose.pose.covariance = [0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0,
                                               0, 0, 0.01, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0,
                                               0, 0, 0, 0, 0, 0]

        self.depth_map_pose.header.frame_id = rospy.get_param('~namespace') + '/description/depth_map_frame'
        self.depth_map_pose.pose.pose.position.z = 0

        depth_odom_pub = rospy.Publisher('cusub_common/depth_odom', PoseWithCovarianceStamped, queue_size=1)
        depth_map_pub = rospy.Publisher('cusub_common/depth_map', PoseWithCovarianceStamped, queue_size=1)

        rate = rospy.Rate(30)

        while not rospy.is_shutdown():
            print "test"

            depth_odom_pub.publish(self.depth_odom_pose)
            depth_map_pub.publish(self.depth_map_pose)

            rate.sleep()

if __name__ == '__main__':
    DEPTH_SENSOR = DepthSensor()
    try:
        DEPTH_SENSOR.depth_sensor()
    except rospy.ROSInterruptException:
        rospy.loginfo("Depth sensor died!")
