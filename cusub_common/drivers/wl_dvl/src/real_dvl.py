#!/usr/bin/env python
from wldvl import WlDVL
import rospy
from geometry_msgs.msg import Vector3Stamped
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np


dvl_pub = rospy.Publisher("dvl",TwistWithCovarianceStamped,queue_size=5)
rospy.init_node('real_dvl')

def load_params():
    # Loads all of the estimated noise parameters
    axes = ["x","y","z"]
    params = {}
    for a in axes:
        params[a + "_bias"] = rospy.get_param("mission_config/dvl_{}/bias".format(a)) # Unused
        params[a + "_var"] = rospy.get_param("mission_config/dvl_{}/var".format(a))
    return params

# params = load_params()

dvl = WlDVL("/dev/dvl")
# dvl = WlDVL("/dev/serial/by-id/usb-FTDI_FT230X_Basic_UART_D30BW6H6-if00-port0")
while not rospy.is_shutdown():
    try:
        report = dvl.read()
    except:
        print("Failed")
        report = None
    # print(report)
    if report is not None:
        if report['valid']:
            print(report)
            t = TwistWithCovarianceStamped()
            t.header.stamp = rospy.get_rostime()
            t.header.frame_id = 'triton/description/base_link'
            t.twist.twist.linear.x = report['vx']
            t.twist.twist.linear.y = -report['vy']
            t.twist.twist.linear.z = -report['vz']


            cov = np.diag([0.1, 0.1, -1, -1, -1, -1])
            t.twist.covariance = list(cov.flatten())

            dvl_pub.publish(t)

