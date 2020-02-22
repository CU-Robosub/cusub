#!/usr/bin/env python
from __future__ import print_function
import rospy

class bcolors: # For terminal colors
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

class CUPrint:
    def __init__(self, name, ros=True):
        self.name = name
        self.ros = ros

    def __call__(self, string, warn=False, print_prev_line=False):
        
        name_str = "[" + bcolors.OKGREEN + self.name + bcolors.ENDC + "] "
        warn_str =  bcolors.WARNING +"[WARN] " + bcolors.ENDC
        print_string = name_str
        if warn:
            print_string = print_string + warn_str
        # if print_prev_line:
            # print_string = "\033[F" + print_string

        if self.ros:
            if print_prev_line:
                print("\033[F", end="")
            rospy.loginfo(print_string + string)
        else:
            if print_prev_line:
                print("\033[F", end="")
            print(print_string + string)