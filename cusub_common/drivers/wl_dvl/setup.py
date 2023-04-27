#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup()
d['packages'] = ['ros_dvl']
d['package_dir'] = {'ros_dvl': 'src/ros_dvl'}

setup(**d)

