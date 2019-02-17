#!/usr/bin/env python
"""

This class simplifies the interface to the pid controller.

To publish to pid loops: 
call self.publishRelativeGoal( [ drive, strafe, depth, yaw ] )

Attribute dictionaries:
- drive
  'state' (float) and 'controlEffort' (float)
- strafe
  'state' and 'controlEffort'
- depth
  'state' and 'controlEffort'
- yaw
  'state' and 'controlEffort'
- roll
  'state' and 'controlEffort'
- pitch
  'state' and 'controlEffort'

NOTE: class that creates this object needs rospy.spin()

"""

__author__ = "luke barbier"
__copyright__ = "Copyright 2018, CU RoboSub"
__license__ = "MIT"
__version__ = "1.1"
__email__ = "luba6098@colorado.edu"
__status__ = "Development"

import rospy
from std_msgs.msg import Float64

class Controller_If():

    def __init__(self):

        # local controller attributes
        self.drive = {}
        self.strafe = {}
        self.depth = {}
        self.yaw = {}
        self.pitch = {}
        self.roll = {}

        baseTopic = '/local_control/pid/'
                
        # set up subscribers for every state pid topic
        s = '/state'
        rospy.Subscriber(baseTopic + 'drive' + s, Float64, self.driveStateCallback)
        rospy.Subscriber(baseTopic + 'strafe' + s, Float64, self.strafeStateCallback)
        rospy.Subscriber(baseTopic + 'depth' + s, Float64, self.depthStateCallback)
        rospy.Subscriber(baseTopic + 'yaw' + s, Float64, self.yawStateCallback)
        rospy.Subscriber(baseTopic + 'pitch' + s, Float64, self.pitchStateCallback)
        rospy.Subscriber(baseTopic + 'roll' + s, Float64, self.rollStateCallback)
        
        # set up subscribers for every controlEffort pid topic
        c = '/control_effort'
        rospy.Subscriber(baseTopic + 'drive' + c, Float64, self.driveEffortCallback)
        rospy.Subscriber(baseTopic + 'strafe' + c, Float64, self.strafeEffortCallback)
        rospy.Subscriber(baseTopic + 'depth' + c, Float64, self.depthEffortCallback)
        rospy.Subscriber(baseTopic + 'yaw' + c, Float64, self.yawEffortCallback)
        rospy.Subscriber(baseTopic + 'pitch' + c, Float64, self.pitchEffortCallback)
        rospy.Subscriber(baseTopic + 'roll' + c, Float64, self.rollEffortCallback)

        # set up private publishers for all setpoints
        st = '/setpoint'
        self._drivePub = rospy.Publisher(baseTopic + 'drive' + st, Float64, queue_size=10)
        self._strafePub = rospy.Publisher(baseTopic + 'strafe' + st, Float64, queue_size=10)
        self._depthPub = rospy.Publisher(baseTopic + 'depth' + st, Float64, queue_size=10)
        self._yawPub = rospy.Publisher(baseTopic + 'yaw' + st, Float64, queue_size=10)
        self._pitchPub = rospy.Publisher(baseTopic + 'pitch' + st, Float64, queue_size=10)
        self._rollPub = rospy.Publisher(baseTopic + 'roll' + st, Float64, queue_size=10)

    def pubDriveGoal(self, dist):
        msg = Float64() ; msg.data = dist
        self._drivePub.publish(msg)

    def pubStrafeGoal(self, dist):
        msg = Float64() ; msg.data = dist
        self._strafePub.publish(msg)

    def pubDepthGoal(self, dist):
        msg = Float64() ; msg.data = dist
        self._depthPub.publish(msg)

    def pubYawGoal(self, angleRad):
        msg = Float64() ; msg.data = angleRad
        self._yawPub.publish(msg)

    def pubPitchGoal(self, angleRad):
        msg = Float64() ; msg.data = angleRad
        self._pitchPub.publish(msg)

    def pubRollGoal(self, angleRad):
        msg = Float64() ; msg.data = angleRad
        self._rollPub.publish(msg)

######################### STATE CALLBACKS ######################################

    def driveStateCallback(self, state):
        self.drive['state'] = state.data

    def strafeStateCallback(self, state):
        self.strafe['state'] = state.data
    
    def depthStateCallback(self, state):
        self.depth['state'] = state.data

    def yawStateCallback(self, state):
        self.yaw['state'] = state.data

    def pitchStateCallback(self, state):
        self.pitch['state'] = state.data 

    def rollStateCallback(self, state):
        self.roll['state'] = state.data

######################### CONTROL EFFORT CALLBACKS ######################################

    def driveEffortCallback(self, effort):
        self.drive['controlEffort'] = effort.data

    def strafeEffortCallback(self, effort):
        self.strafe['controlEffort'] = effort.data
    
    def depthEffortCallback(self, effort):
        self.depth['controlEffort'] = effort.data

    def yawEffortCallback(self, effort):
        self.yaw['controlEffort'] = effort.data

    def pitchEffortCallback(self, effort):
        self.pitch['controlEffort'] = effort.data 

    def rollEffortCallback(self, effort):
        self.roll['controlEffort'] = effort.data

if __name__ == "__main__":
    rospy.init_node('test_controller_interface')
    c = Controller_If()
    rospy.loginfo("No syntax errors")
    rospy.spin()
