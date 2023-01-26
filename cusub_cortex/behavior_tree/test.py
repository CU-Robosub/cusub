#!/usr/bin/env python

import behavior as bt
import rospy
import time
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Bool


class Test:   
    def __init__(self):
        # constants
        self.rate = 1.5
        #self.delta = 1.0 / self.rate
        #self.speed = 0.8

    # create nodes for behaivor tree
    root = bt.Selector("root")

    condition = bt.Condition("condition", 'should_move')
    sequence = bt.Sequence("sequence", True)
    move1 = bt.Move("move1", 'position')
    move2 = bt.Move("move2", 'position')

    move3 = bt.Move("move3", 'position')


    # arrange nodes
    root.nodes.append(condition)
    condition.nodes.append(sequence)
    sequence.nodes.append(move1)
    sequence.nodes.append(move2)

    root.nodes.append(move3)

    # display behaivor tree
    bt.display_tree(root)


    # set up publisher
    def publisher(self):
        # create node and publisher
        rospy.init_node('state_machine')
        pub = rospy.Publisher('/leviathan/cusub_common/motor_controllers/pid/drive/setpoint', Float64, queue_size=1)
        rospy.Subscriber("/leviathan/cusub_common/motor_controllers/pid/drive/state", Float64, self.callback)
        rospy.Subscriber("move", Bool, self.callback2)

        self.move1.publisher = pub
        self.move2.publisher = pub
        self.move3.publisher = pub

        self.move1.destination = 3.0
        self.move2.destination = -3.0
        self.move3.destination = 0.0
        
        # set rate
        rate = rospy.Rate(self.rate)
        
        # main loop
        while not rospy.is_shutdown():

            self.root.activate(bt.blackboard)
            # wait
            rate.sleep()

    def callback(self, data):
        bt.blackboard['position'] = round(data.data, 1)
    def callback2(self, data):
        bt.blackboard['should_move'] = data.data



test = Test()
test.publisher()