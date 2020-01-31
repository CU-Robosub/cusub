#!/usr/bin/env python

import yaml
import os

# Check we're in the file path we expect
folder = os.getcwd()
if folder[-7:] != "scripts":
    print("Please run place_priors.py in the scripts folder")

# cd into the config folder
os.chdir("../config")

# Open the data
data = None
with open('mission_config.yaml') as f:
    data = yaml.load(f, Loader=yaml.FullLoader)

# Modify priors
priors = {}
for task in data["tasks"].keys():
    if task == "startup":
        continue
    priors[task] = [0.0, 0.0, 0.0]
    data["tasks"][task]["prior"] = [0.0, 0.0, 0.0]

# TODO decide if we want to create a sim OR real mission config
# TODO output the sub's starting world location
# Dump the data into mission_config_noisy
with open('mission_config_noisy.yaml', 'w') as f:    
    yaml.dump(data, f)

# cd into the simulation
os.chdir("../../../cusub_sim/cusub_sim_bringup/config/")
with open('model_locations.yaml', 'w') as f:
    yaml.dump(priors, f)