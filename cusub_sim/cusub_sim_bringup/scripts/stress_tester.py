#!/usr/bin/env python

import rospy
import os
import yaml
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import tf

def set_model_state(model_name, pose):
    for i in range(3): # repeat 3 times, sometimes gazebo doesn't actually move the model but indicates it does in its modelstate...
        rospy.wait_for_service('/gazebo/set_model_state')
        result = None
        try:
            mover = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            res = mover( ModelState(model_name, pose, Twist(), "world") )
            # add_two_ints = rospy.ServiceProxy('add_two_ints', AddTwoInts)
            # resp1 = add_two_ints(x, y)
            # return resp1.sum
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)

def main():
    # Check we're in the file path we expect
    folder = os.getcwd()
    if folder[-7:] != "scripts":
        print("Please run stress_tester.py in the scripts folder")

    # cd into the config folder
    os.chdir("../config")

    # Open the data
    model_locs = None
    with open('model_locations.yaml') as f:
        model_locs = yaml.load(f, Loader=yaml.FullLoader)
    params = None
    with open('stress_tester.yaml') as f:
        params = yaml.load(f, Loader=yaml.FullLoader)

    # Let's see if I can spawn the start gate!

    # check if the start gate exists in gazebo
    # --> delete it

    print(params)
    print(model_locs)
    sg_params = params["tasks"]["start_gate"]
    sg = model_locs["start_gate"]

    # spawn the start gate with random noise in position + orientation

    # Add noise to the pose
    yaw = sg_params["yaw"]
    quat = tf.transformations.quaternion_from_euler(0,0, yaw)
    ori = Quaternion(quat[0],quat[1],quat[2],quat[3])
    x, y, z = sg[0], sg[1], sg[2]
    pose_in_world = Pose(Point(x,y,z), ori)

    model_name = sg_params["model"] + "_1"

    # Just move the object in gazebo, let's not deal with this deleting and spawning...
    set_model_state(model_name, pose_in_world)
    # req = ModelState(model_name, pose_in_world, Twist(), "world")

if __name__ == "__main__":
    rospy.init_node("stress_tester")
    main()