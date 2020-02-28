#!/usr/bin/env python
from __future__ import division
import rospy
import os
import yaml
import numpy as np
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState
import tf
import argparse
import time
from cusub_print.cuprint import CUPrint, bcolors

cuprint = CUPrint("Gazebo Mover", ros=False)

def set_model_state(model_name, pose):
    rospy.wait_for_service('/gazebo/set_model_state')    
    for i in range(3): # repeat 3 times, sometimes gazebo doesn't actually move the model but indicates it does in its modelstate...    
        result = None
        try:
            mover = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
            res = mover( ModelState(model_name, pose, Twist(), "world") )
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s", e)
        time.sleep(0.1)

def main(noise, gazebo_paused):
    # Check we're in the file path we expect
    folder = os.getcwd()
    if folder[-7:] != "scripts":
        cuprint("Please run gazebo.py in the scripts folder")

    # cd into the config folder
    os.chdir("../config")

    # Open the data
    model_locs = None
    with open('model_locs.yaml') as f:
        model_locs = yaml.load(f, Loader=yaml.FullLoader)
    params = None
    with open('stress_tester.yaml') as f:
        params = yaml.load(f, Loader=yaml.FullLoader)["sim_gt"]

    # Load defualt noise params
    x_default_noise = params["default_x_noise_std"]
    y_default_noise = params["default_y_noise_std"]
    z_default_noise = params["default_z_noise_std"]
    yaw_default_noise = (np.pi/180) * params["default_yaw_noise_std_deg"]

    if not gazebo_paused:
        raw_input("Gazebo paused?")

    noisy_positions = {}

    # loop through all objects in model_locs.yaml
    for model in model_locs.keys():
        if model == "mappings": # just add it and continue
            noisy_positions[model] = model_locs[model]
            continue

        [x, y, z, yaw] = model_locs[model]
        if noise:
            # Check for additional noise
            x += np.random.normal(0, x_default_noise)
            y += np.random.normal(0, y_default_noise)
            z += np.random.normal(0, z_default_noise)
            yaw += np.random.normal(0, yaw_default_noise)

        noisy_positions[model] = [x,y,z,yaw]

        # Generate model's pose
        quat = tf.transformations.quaternion_from_euler(0,0, yaw)
        ori = Quaternion(quat[0],quat[1],quat[2],quat[3])
        pose_in_world = Pose(Point(x,y,z), ori)

        cuprint("moving " + bcolors.HEADER + model + bcolors.ENDC)
        set_model_state(model, pose_in_world)

    with open('noisy_model_locs.yaml', 'w') as f:    
        yaml.dump(noisy_positions, f)

""" Accept all types of boolean commandline input """
def str2bool(v):
    if isinstance(v, bool):
        return v
    if v.lower() in ('yes', 'true', 't', 'y', '1'):
        return True
    elif v.lower() in ('no', 'false', 'f', 'n', '0'):
        return False
    else:
        raise argparse.ArgumentTypeError('Boolean value expected.')

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Moves gazebo models to match config/model_locs.yaml. Optionally adds noise to the locations.')
    parser.add_argument("-n", "--noise", type=str2bool, default=False, help='bool, add noise to the gazebo model locations. Default: true')
    parser.add_argument("-gp", "--gazebo_paused", type=str2bool, default=False, help='bool, indicate if gazebo is already paused. Default: false')
    args = parser.parse_args()
    if args.noise:
        cuprint("adding noise to gazebo locations")
    else:
        cuprint("NOT adding noise to gazebo locations")

    rospy.init_node("stress_tester")

    main(args.noise, args.gazebo_paused)