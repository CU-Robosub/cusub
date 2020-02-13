#!/usr/bin/env python
from __future__ import division
import rospy
import os
import yaml
import numpy as np
import argparse
from cusub_print.cuprint import CUPrint, bcolors

cuprint = CUPrint("Prior Generator", ros=False)

def calculate_correct_priors(model_locs):
    priors = {}

    try:
        mappings = model_locs["mappings"]
        leviathan = model_locs["leviathan/description"]
    except KeyError as e:
        cuprint(bcolors.FAIL + "Incorrectly formatted model_locs_noisy.yaml, missing: " + str(e) + bcolors.ENDC)
        return None
    theta = leviathan[3] # sub's orientation in world frame
    for task in mappings.keys():
        cuprint("calculating " + bcolors.HEADER + task + bcolors.ENDC + " prior")
        model = mappings[task]["model"]
        xdiff = model_locs[model][0] - leviathan[0]
        ydiff = model_locs[model][1] - leviathan[1]
        psi = np.arctan2(ydiff, xdiff)
        alpha = psi - theta
        rho = np.linalg.norm([xdiff, ydiff])

        x_prior = float( rho * np.cos(alpha) )
        y_prior = float( rho * np.sin(alpha) )
        
        if "prior_z" in mappings[task].keys():
            z_prior = float( mappings[task]["prior_z"] )
        else:
            z_prior = float( model_locs[model][2] )
        
        # Nice numbers in the config
        x_prior = round(x_prior, 2)
        y_prior = round(y_prior, 2)
        z_prior = round(z_prior, 2)
        
        priors[task] = [x_prior, y_prior, z_prior]
    return priors

def apply_noise(priors, leviathan, params, gauss=True, x_bias=True, y_bias=True, yaw_bias=True):
    """
    Applies noise to the priors

    params
    ------
    priors : dict
    leviathan : list
        [x,y,z,yaw]
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

    # loop through all tasks in priors
    x_bias = np.random.normal(0, 4)
    y_bias = np.random.normal(0, 4)
    yaw_bias = np.random.normal(0,0.3)
    rotation_matrix = np.array( \
        [[np.cos(yaw_bias), -np.sin(yaw_bias)],
        [np.sin(yaw_bias), np.cos(yaw_bias)]]
        )
    lev_x, lev_y = leviathan[0], leviathan[1]

    noisy_priors = {}
    for task in priors.keys():
        [x, y, z] = priors[task]
        if gauss:
            x += np.random.normal(0, x_default_noise)
            y += np.random.normal(0, y_default_noise)
            z += np.random.normal(0, z_default_noise)
        if x_bias:
            x += x_bias
        if y_bias:
            y += y_bias
        if yaw_bias:
            # apply rotation on the prior
            rotated_coords = np.dot( rotation_matrix, np.array([[x],[y]]) )
            x = rotated_coords[0,0]
            y = rotated_coords[1,0]
        x = round( float(x), 2)
        y = round( float(y), 2)
        z = round( float(z), 2)
        noisy_priors[task] = [x,y,z]
    return noisy_priors

def write_mission_config(mission_params, priors, new_file_name):
    for task in priors.keys():
        mission_params["tasks"][task]["prior"] = priors[task]
    with open(new_file_name, 'w') as f:    
        yaml.dump(mission_params, f)

def main(noise, gauss, x_bias, y_bias, yaw_bias):
    # Check we're in the file path we expect
    folder = os.getcwd()
    if folder[-7:] != "scripts":
        cuprint("Please run priors.py in the scripts folder")
    
    # cd into the config folder
    os.chdir("../config")

    # STEP 1: Open the data & calculate the correct priors
    model_locs = None
    with open('noisy_model_locs.yaml') as f:
        model_locs = yaml.load(f, Loader=yaml.FullLoader)
    priors = calculate_correct_priors(model_locs)
    if priors == None:
        return

    # STEP 2: Apply noise to the priors
    params = None
    with open('stress_tester.yaml') as f:
        params = yaml.load(f, Loader=yaml.FullLoader)["priors"]
    leviathan = model_locs["leviathan/description"]
    if noise:
        cuprint("adding noise to priors")
        noisy_priors = apply_noise(priors, leviathan, params, gauss, x_bias, y_bias, yaw_bias)
    else:
        cuprint("NOT adding noise to priors")
        noisy_priors = priors

    # STEP 3: write the new mission_config
    filename = '../../../cusub_cortex/state_machine/config/mission_config_generated.yaml'
    with open(filename) as f:
        mission_params = yaml.load(f, Loader=yaml.FullLoader)
    new_file_name = '../../../cusub_cortex/state_machine/config/noisy_mission_config_generated.yaml'
    cuprint("creating config file: " + bcolors.HEADER + "noisy_mission_config_generated.yaml" + bcolors.ENDC)
    write_mission_config(mission_params, noisy_priors, new_file_name)

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
    parser.add_argument("-x", "--x_noise", type=str2bool, default=False, help='bool, adds translational x noise to all priors. Default: true')
    parser.add_argument("-y", "--y_noise", type=str2bool, default=False, help='bool, adds translational y noise to all priors. Default: true')
    parser.add_argument("-yaw", "--yaw_noise", type=str2bool, default=False, help="bool, adds rotational noise to all priors about leviathan's truth pose. Default: true")
    args = parser.parse_args()
    main(args.noise, args.gaussian, args.x_noise, args.y_noise,args.yaw_noise)