#!/usr/bin/env python

import enum

# each node returns one of these statuses when activated
class Status(enum.Enum):
    
    SUCCESS = 1
    RUNNING = 2
    FAILURE = 3

# basic node class that everything else inherits from
class Node:
    
    def __init__(self, name):
        self.name = name
    
    def activate(self):
        return Status.SUCCESS

# 2 dimensional vector, used for velocity
class Vector2():
    def __init__(self, x, y):
        self.x = x
        self.y = y

    def __add__(self, other):
        return Vector2(self.x + other.x, self.y + other.y)

    def __sub__(self, other):
        return Vector2(self.x - other.x, self.y - other.y)

    def __mul__(self, other):
        return Vector2(self.x * other.x, self.y * other.y)

    def __div__(self, other):
        return Vector2(self.x / other.x, self.y / other.y)

    def __float__(self):
        return self.x

    def normalized(self, speed = 1.0):
        self.length = (self.x**2 + self.y**2 + self.z**2)**0.5
        return Vector2(self.x / self.length * speed, self.y / self.length * speed)

# this stores all the information needed for the robot
blackboard = {
        "rate" : 2, # this should stay constant, don't change it in the code
        "drive_publisher" : None,
        "strafe_publisher" : None,
        "depth_publisher" : None,
        "yaw_publisher" : None,
        "velocity" : Vector2(0.5, 0.0), # (Drive, Strafe)
        "depth" : -0.2,
        "yaw" : 0.0,
        "counter" : 0
}

def is_number(s):
    try:
        float(s)
        return True
    except ValueError:
        return False





#######################################################################################
'''

# displays a node and all of its descendents. not necessary now that the visual editor works
def display_tree(root):
    print(root.name)
    iterate(root, 1)
        
def iterate(node, level):
    for i in node.nodes:
        print(level * "  " + i.name)
        if hasattr(i, "nodes") and len(i.nodes) > 0:
            iterate(i, level + 1)
'''