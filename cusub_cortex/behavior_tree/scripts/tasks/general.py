#!/usr/bin/env python

import enum
import rospy
import math_utilities as mu
from detection_listener.listener import DetectionListener

# each node returns one of these statuses when activated
class Status(enum.Enum):
    
    SUCCESS = 1
    RUNNING = 2
    FAILURE = 3

# this stores all the information needed for the robot
blackboard = {
        "rate" : 2, # this should stay constant, don't change it in the code
        "drive_publisher" : None,
        "strafe_publisher" : None,
        "velocity" : mu.Vector2(1.0, 0.0),
        "counter" : 1
    }

# basic node class that everything else inherits from
class Node:
    
    def __init__(self, name):
        self.name = name
    
    def activate(self, board):
        return Status.SUCCESS


# activates each child node until one of them returns RUNNING or FAILURE, then returns the same, returns SUCCESS if it makes it all the way through, can also be used for root node
class Sequence(Node):
    
    def __init__(self, name, save_running = False):
        self.name = name
        self.nodes = []
        self.running_node = 0
        self.save_running = save_running # if save_running is true, it will activate starting from the node that previously returned RUNNING (this might be removed, but for now it is useful for testing stuff)
    
    def activate(self, board):
        if self.save_running:
            node_range = range(self.running_node, len(self.nodes))
        else:
            node_range = range(0, len(self.nodes))
        
        for i in node_range:
            self.status = self.nodes[i].activate(board)
            if self.status == Status.RUNNING:
                self.running_node = i
                return self.status
            elif self.status == Status.FAILURE:
                self.running_node = 0
                return self.status
            
        self.running_node = 0
        return Status.SUCCESS
            

# activate each child node until one of them returns RUNNING or SUCCESS, then returns the same, returns FAILURE if it makes it all the way through, can also be used for root node
class Selector(Node):
    
    def __init__(self, name):
        self.name = name
        self.nodes = []
    
    def activate(self, board):
        for i in range(0, len(self.nodes)):
            self.status = self.nodes[i].activate(board)
            if self.status == Status.RUNNING:
                return self.status
            elif self.status == Status.SUCCESS:
                return self.status
            
        return Status.FAILURE

# returns the same as the child node if condition is true and FAILURE if condition is false
class Condition(Node):
    
    def __init__(self, name, blackboard_variable, desired_value = True):
        self.name = name
        self.blackboard_variable = blackboard_variable
        self.desired_value = desired_value # the value the node is checking against
        self.nodes = [] # conditionals should only have 1 child node
    
    def activate(self, board):
        if board[self.blackboard_variable] == self.desired_value:
            if len(self.nodes) == 0:
                return Status.SUCCESS
            if len(self.nodes) == 1:
                return self.nodes[0].activate(board)
            else:
                print(self.name + " has too many child nodes, it should only have 1")
                return Status.FAILURE
        else:
            return Status.FAILURE

# modifies the value of a blackboard variable, returns FAILURE if variable does not exist or if the value is not the same type and returns SUCCESS otherwise
class Set(Node):

    def __init__(self, name, variable, value, operator = 0):
        self.name = name
        self.variable = variable
        self.value = value
        self.operator = operator # 0 sets the value, 1 adds the value, 2 multiplies the value

    def activate(self, board):
        if self.variable in board:
            if type(board[self.variable]) == type(self.value):
                if self.operator == 0:
                    board[self.variable] = self.value
                if self.operator == 1:
                    board[self.variable] += self.value
                if self.operator == 2:
                    board[self.variable] *= self.value

            return Status.SUCCESS


# new nodes can be created for new behavior, this is an example of how to create one
class Example(Node):

    def __init__(self, name):
        self.name = name
    
    def activate(self, board):
        print(self.name)
        return Status.SUCCESS


# displays a node and all of its descendents
def display_tree(root):
    print(root.name)
    iterate(root, 1)
        
def iterate(node, level):
    for i in node.nodes:
        print(level * "  " + i.name)
        if hasattr(i, "nodes") and len(i.nodes) > 0:
            iterate(i, level + 1)