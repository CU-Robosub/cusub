#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistWithCovarianceStamped
import numpy as np


rospy.init_node("dvl")
pub = rospy.Publisher("dvl", TwistWithCovarianceStamped, queue_size=2)

def load_params():
    # Loads all of the estimated noise parameters
    axes = ["x","y","z"]
    params = {}
    for a in axes:
        params[a + "_bias"] = rospy.get_param("mission_config/dvl_{}/bias".format(a)) # Unused
        params[a + "_var"] = rospy.get_param("mission_config/dvl_{}/var".format(a))
    return params

params = load_params()

def callback(msg):
    global pub, params

    namespace = rospy.get_namespace().replace("/","")

    t = TwistWithCovarianceStamped()
    t.header.stamp = rospy.get_rostime()
    t.header.frame_id = 'base_link'
    t.twist = msg.twist
    t.twist.twist.linear.x += np.random.normal(0,0.1)
    t.twist.twist.linear.y += np.random.normal(0,0.1)
    t.twist.twist.linear.z += np.random.normal(0,0.1)

    cov = np.diag([params["x_var"], params["y_var"], params["z_var"], -1, -1, -1])
    t.twist.covariance = list(cov.flatten())

    pub.publish(t)

rospy.Subscriber("pose_gt", Odometry, callback, queue_size=1)
rospy.spin()