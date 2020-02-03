#!/usr/bin/env python
from __future__ import division
import rospy
import os
import yaml
import numpy as np
import argparse

class bcolors: # For terminal colors
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def calculate_correct_priors(model_locs):
    priors = {}

    try:
        mappings = model_locs["mappings"]
        leviathan = model_locs["leviathan/description"]
    except KeyError as e:
        print(bcolors.FAIL + "Incorrectly formatted model_locs_noisy.yaml, missing: " + str(e) + bcolors.ENDC)
        return None
    theta = leviathan[3] # sub's orientation in world frame
    for task in mappings.keys():
        print("calculating " + bcolors.HEADER + task + bcolors.ENDC + " prior")
        model = mappings[task]["model"]
        xdiff = model_locs[model][0] - leviathan[0]
        ydiff = model_locs[model][1] - leviathan[1]
        psi = np.arctan2(ydiff, xdiff)
        alpha = psi - theta
        rho = np.linalg.norm([xdiff, ydiff])

        x_prior = float( rho * np.cos(alpha) )
        y_prior = float( rho * np.sin(alpha) )
        
        if "fixed_z" in mappings[task].keys():
            z_prior = float( mappings[task]["fixed_z"] )
        else:
            z_prior = float( model_locs[model][2] )
        
        # Nice numbers in the config
        x_prior = round(x_prior, 2)
        y_prior = round(y_prior, 2)
        z_prior = round(z_prior, 2)
        
        priors[task] = [x_prior, y_prior, z_prior]
    return priors

def apply_noise(priors, params, gauss=True, x_bias=True, y_bias=True, yaw_bias=True):
    """
    Applies noise to the priors

    params
    ------
    priors : dict
    params : dict
    gauss : bool
        apply zero-mean gaussian random noise to all priors that don't indicate not to params?
    x_bias : bool
        apply an x translation to all params?
    y_bias : bool
        apply an x translation to all params?
    yaw_bias : bool
        apply a rotation about leviathan's location?

    returns
    -------
    noisy_params : dict
    """
    # Load defualt noise params
    x_default_noise = params["default_x_noise_std"]
    y_default_noise = params["default_y_noise_std"]
    z_default_noise = params["default_z_noise_std"]
    yaw_default_noise = (np.pi/180) * params["default_yaw_noise_std_deg"]

    # loop through all objects in model_locs.yaml
    for model in model_locs.keys():
        [x, y, z, yaw] = model_locs[model]
        if noise:
            # Check for additional noise
            x += np.random.normal(0, x_default_noise)
            y += np.random.normal(0, y_default_noise)
            z += np.random.normal(0, z_default_noise)
            yaw += np.random.normal(0, yaw_default_noise)

def write_mission_config(mission_params, priors, new_file_name):
    for task in priors.keys():
        mission_params["tasks"][task]["prior"] = priors[task]
    with open(new_file_name, 'w') as f:    
        yaml.dump(mission_params, f)

def main(noise, gauss, x_bias, y_bias, yaw_bias):
    # Check we're in the file path we expect
    folder = os.getcwd()
    if folder[-7:] != "scripts":
        print("Please run priors.py in the scripts folder")
    
    # cd into the config folder
    os.chdir("../config")

    # STEP 1: Open the data & calculate the correct priors
    model_locs = None
    with open('model_locs_noisy.yaml') as f:
        model_locs = yaml.load(f, Loader=yaml.FullLoader)
    priors = calculate_correct_priors(model_locs)
    if priors == None:
        return

    # STEP 2: Apply noise to the priors
    params = None
    with open('stress_tester.yaml') as f:
        params = yaml.load(f, Loader=yaml.FullLoader)["priors"]
    if noise:
        print("adding noise to priors")
        noisy_priors = apply_noise(priors, params, gauss, x_bias, y_bias, yaw_bias)
    else:
        print("NOT adding noise to priors")
        noisy_priors = priors

    # STEP 3: write the new mission_config
    with open('active_mission_config.yaml') as f:
        mission_params = yaml.load(f, Loader=yaml.FullLoader)
    active_mission_config = os.path.realpath("active_mission_config.yaml").split("/")[-1]
    tmp = active_mission_config.split(".")
    new_file_name = tmp[0] + "_noisy." + tmp[1]
    print("creating config file: " + bcolors.HEADER + new_file_name + bcolors.ENDC)
    write_mission_config(mission_params, noisy_priors, new_file_name)

    # STEP 4: Update the state machine's symlink
    print("updating symlink in state_machine: " + bcolors.HEADER + "noisy_mission_config.yaml" + bcolors.ENDC)
    src = "../../../cusub_sim/stress_tester/config/" + new_file_name
    dst = "../../../cusub_cortex/state_machine/config/noisy_mission_config.yaml"
    os.symlink(src, dst + ".tmp")
    os.rename(dst + ".tmp", dst) # rename to overwrite existing file


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
    parser = argparse.ArgumentParser(description='Updates priors for a mission_config. Optionally adds noise to priors.')
    parser.add_argument("-n", "--noise", type=str2bool, default=True, help='bool, add noise to priors. Default: true')
    parser.add_argument("-g", "--gaussian", type=str2bool, default=True, help='bool, add gaussian noise to all priors. Default: true')
    parser.add_argument("-x", "--x_noise", type=str2bool, default=True, help='bool, adds translational x noise to all priors. Default: true')
    parser.add_argument("-y", "--y_noise", type=str2bool, default=True, help='bool, adds translational y noise to all priors. Default: true')
    parser.add_argument("-yaw", "--yaw_noise", type=str2bool, default=True, help="bool, adds rotational noise to all priors about leviathan's truth pose. Default: true")
    args = parser.parse_args()
    main(args.noise, args.gaussian, args.x_noise, args.y_noise,args.yaw_noise)