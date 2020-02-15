#!/usr/bin/env python
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

    def __call__(self, string, warn=False):
        if self.ros:
            if warn:
                rospy.loginfo("[" + bcolors.OKGREEN + self.name + bcolors.ENDC + "] " + bcolors.WARNING +"[WARN] "+ string + bcolors.ENDC)
            else:
                rospy.loginfo("[" + bcolors.OKGREEN + self.name + bcolors.ENDC + "] " + string)
        else:
            if warn:
                print("[" + bcolors.OKGREEN + self.name + bcolors.ENDC + "] " + bcolors.WARNING +"[WARN] "+ string + bcolors.ENDC)
            else:
                print("[" + bcolors.OKGREEN + self.name + bcolors.ENDC + "] " + string)