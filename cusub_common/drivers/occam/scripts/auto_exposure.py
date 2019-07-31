#!/usr/bin/env python

import dynamic_reconfigure.client
import rospy
from std_msgs.msg import Float64

RANGE_LO = rospy.get_param('occam/range_lo')
RANGE_HI = rospy.get_param('occam/range_hi')
RANGE = RANGE_HI - RANGE_LO
NUM_UPDATES = rospy.get_param('occam/num_updates')
PERIOD = rospy.get_param('occam/update_period')
INCREMENT = (RANGE_HI-RANGE_LO) / NUM_UPDATES
STEP_WAIT = PERIOD / NUM_UPDATES

set_exp_pub = rospy.Publisher("leviathan/set_exposure", Float64, queue_size=1)
exp_set = RANGE_LO - INCREMENT

rospy.init_node('auto_exposure', anonymous=True)
# TODO: add node names for the cameras
# downcam = dynamic_reconfigure.client.Client("/camera/camera_nodelet/exposure")
# torpcam = dynamic_reconfigure.client.Client()

while not rospy.is_shutdown():
    exp_set += INCREMENT
    if exp_set > RANGE_HI:
        exp_set = RANGE_LO
    msg = Float64()
    msg.data = exp_set
    set_exp_pub.publish(msg)
    # TODO: create dict for downcam
    # params = { '' : exp_set/RANGE}
    # downcam.update_configuration(params)
    # TODO: update for torpcam
    # params = { '' : exp_set}
    # torpcam.update_configuration(params)
    rospy.loginfo("current exposure is: %f", exp_set)
    rospy.sleep(STEP_WAIT)
