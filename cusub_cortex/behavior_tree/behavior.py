#!/usr/bin/env python

import enum
import rospy

# each node returns one of these statuses when activated
class Status(enum.Enum):
    
    SUCCESS = 1
    RUNNING = 2
    FAILURE = 3

# the blackboard stores all the information needed for the robot
blackboard = {
        'position' : 0.0,
        'should_move' : False
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
                print(self.name + ' has too many child nodes, it should only have 1')
                return Status.FAILURE
        else:
            return Status.FAILURE

# class Inverter(Node):

#     def __init__(self, name):
#         self.name = name
#         self.nodes = [] # inverters should only have 1 child node

#     def activate(self):
#         if len(self.nodes) == 0:
#             print(self.name + ' has no child node')
#             return Status.FAILURE
#         if len(self.nodes) == 1:
#             return not self.nodes[0].activate(board)
#         else:
#             print(self.name + ' has too many child nodes, it should only have 1')
#             return Status.FAILURE


# moves robot to destination, returns RUNNING while moving and SUCCESS once it reaches it
class Move(Node):

    def __init__(self, name, blackboard_variable, destination = 0.0):
        self.name = name
        self.blackboard_variable = blackboard_variable
        self.destination = destination
        self.publisher = None

    def activate(self, board):
        if self.publisher != None:
            if board[self.blackboard_variable] == self.destination:
                print(self.name + " Robot is already at position " + str(board[self.blackboard_variable]))
                return Status.SUCCESS
            else:
                self.publisher.publish(self.destination)
                print(self.name + " is moving the robot towards " + str(self.destination) + "(current position is " + str(board[self.blackboard_variable]) + ")")
                return Status.RUNNING
        else:
            return Status.FAILURE

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
        if hasattr(i, 'nodes') and len(i.nodes) > 0:
            iterate(i, level + 1)