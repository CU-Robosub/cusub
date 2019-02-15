#!/usr/bin/env python

import rospy

from sensor_msgs.msg import FluidPressure
from geometry_msgs.msg import PoseWithCovarianceStamped

depth_pose = None

def pressure_callback(data):

    global depth_pose

    depth_pose.header.stamp = rospy.Time.now()
    depth_pose.header.seq += 1

    # 103.2 = 8.748m

    depth = (data.fluid_pressure - 103.2) / 9.91

    # make depth resolution 8.1cm
    depth = round(depth / 0.081) * 0.081 - 0.85

    depth_pose.pose.pose.position.z = depth

def depth_sensor():

    global depth_pose

    rospy.init_node('depth_sensor_gazebo', anonymous=True)
    rospy.Subscriber("/leviathan/pressure", FluidPressure, pressure_callback)

    depth_pose = PoseWithCovarianceStamped()
    depth_pose.pose.covariance = [0, 0, 0, 0, 0, 0,
					      0, 0, 0, 0, 0, 0,
					      0, 0, 0.01 , 0, 0, 0,
					      0, 0, 0, 0, 0, 0,
					      0, 0, 0, 0, 0, 0,
					      0, 0, 0, 0, 0, 0]
    depth_pose.header.frame_id = "depth_frame"
    depth_pose.pose.pose.position.z = 0

    depth_pub = rospy.Publisher('depth', PoseWithCovarianceStamped, queue_size=1)

    rate = rospy.Rate(30)

    while not rospy.is_shutdown():

	#depth_pose.header.stamp = rospy.Time.now()
        depth_pub.publish(depth_pose)

        rate.sleep()

if __name__ == '__main__':
    try:
        depth_sensor()
    except rospy.ROSInterruptException:
        rospy.loginfo("Mother fucker!")
