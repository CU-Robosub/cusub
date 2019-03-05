#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

class bangbang():

    def __init__(self):
        pass

    def stateCallback(self, state):

        self.state = state.data;

        effort = Float64()
        effort.data = 0.0

        if(self.state < self.setpoint - self.deadZoneSize / 2):
            effort.data = self.leftBangEffort

        elif(self.state > self.setpoint + self.deadZoneSize / 2):
            effort.data = self.rightBangEffort

        self.controlEffortPub.publish(effort)

    def setpointCallback(self, setpoint):

        self.setpoint = setpoint.data

    def run(self):

        self.setpointTopic = rospy.get_param("~setpoint_topic")
        self.stateTopic = rospy.get_param("~state_topic")
        self.controlEffortTopic = rospy.get_param("~control_effort_topic")

        self.deadZoneSize = rospy.get_param("~deadzone_size", 1.0)

        self.leftBangEffort = rospy.get_param("~left_bang_effort", 1.0)
        self.rightBangEffort = rospy.get_param("~right_bang_effort", -1.0)

        self.setpointSub = rospy.Subscriber(self.setpointTopic, Float64, self.setpointCallback)
        self.stateSub = rospy.Subscriber(self.stateTopic, Float64, self.stateCallback)

        self.controlEffortPub = rospy.Publisher(self.controlEffortTopic, Float64, queue_size=1)

        # TODO timer incase we dont get state for a while

        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('bangbang', anonymous=True)
    a = bangbang()
    try:
        a.run()
    except rospy.ROSInterruptException:
      rospy.logerr("bangbang has died!");
