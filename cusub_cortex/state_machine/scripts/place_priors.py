#!/usr/bin/env python

import yaml
import os

# Check we're in the file path we expect
folder = os.getcwd()
if folder[-7:] != "scripts":
    print("Please run place_priors.py in the scripts folder")

# CD into the config folder
os.chdir("../config")

# Open the data
data = None
with open('mission_config.yaml') as f:
    data = yaml.load(f, Loader=yaml.FullLoader)

# print(data)
for task in data["tasks"].keys():
    data["tasks"][task]["prior"] = [0.0, 0.0, 0.0]

with open('mission_config_noisy.yaml', 'w') as f:    
    yaml.dump(data, f)