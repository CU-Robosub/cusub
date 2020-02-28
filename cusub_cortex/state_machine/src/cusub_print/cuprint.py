#!/usr/bin/env python
from __future__ import print_function
import rospy
import sys

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

    def __call__(self, string, warn=False, err=False, print_prev_line=False):
        
        name_str = "[" + bcolors.OKGREEN + self.name + bcolors.ENDC + "] "
        warn_str = bcolors.WARNING + "[WARN] "
        err_str = bcolors.FAIL + "[FAIL] " 
        print_string = name_str

        if warn:
            print_string = print_string + warn_str + string + bcolors.ENDC
        elif err:
            print_string = print_string + err_str + string + bcolors.ENDC
        else:
            print_string = print_string + string

        if self.ros:
            if print_prev_line:
                print("\033[F\33[2K", end="")
            rospy.loginfo(print_string)
        else:
            if print_prev_line:
                print("\033[F\33[2K", end="")
            print(print_string)
        
        if print_prev_line:
            sys.stdout.flush() # flush stdout before we delay

if __name__ == "__main__":
    rospy.init_node("testing_print")
    cuprint = CUPrint("Test Print Node", ros=False)
    cuprint("I love strawberries")
    cuprint("I love strawberries", warn=True)
    cuprint("I love strawberries", err=True)
    print("string to be overwritten")
    for i in range(10):
        cuprint("num: " + str(i), print_prev_line=True)

        rospy.sleep(0.2)