#!/usr/bin/env python

from general import *

# activates each child node until one of them returns RUNNING or FAILURE, then returns the same, returns SUCCESS if it makes it all the way through, can also be used for root node
class Sequence(Node):
    
    def __init__(self, name, save_running = False):
        self.name = name
        self.nodes = []
        self.running_node = 0
        self.save_running = save_running # if save_running is true, it will activate starting from the node that previously returned RUNNING (this might be removed, but for now it is useful for testing stuff)
    
    def activate(self):
        if self.save_running:
            node_range = range(self.running_node, len(self.nodes))
        else:
            node_range = range(0, len(self.nodes))
        
        for i in node_range:
            self.status = self.nodes[i].activate()
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
    
    def activate(self):
        for i in range(0, len(self.nodes)):
            self.status = self.nodes[i].activate()
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
    
    def activate(self):
        if blackboard[self.blackboard_variable] == self.desired_value:
            if len(self.nodes) == 0:
                return Status.SUCCESS
            if len(self.nodes) == 1:
                return self.nodes[0].activate()
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

    def activate(self):
        if not is_number(self.value):
            self.value = blackboard[self.value]
        if self.variable in blackboard:
            if type(blackboard[self.variable]) == type(self.value):
                if self.operator == 0:
                    blackboard[self.variable] = self.value
                if self.operator == 1:
                    blackboard[self.variable] += self.value
                if self.operator == 2:
                    blackboard[self.variable] *= self.value

            return Status.SUCCESS