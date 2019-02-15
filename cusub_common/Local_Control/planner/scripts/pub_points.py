#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose

rospy.init_node('pub_points_node')

rospy.sleep(15)

pubTopic = '/local_control/planner_goal'
subTopic = '/reached_goal'

pub = rospy.Publisher(pubTopic, Pose, queue_size=10)

startGate = [-6, 18.5, -1]

msg = Pose()
msg.position.x = startGate[0]
msg.position.y =startGate[1]
msg.position.z =startGate[2]

rospy.loginfo("publishing start gate")

pub.publish(msg)
msg = rospy.wait_for_message(subTopic, Empty)
rospy.sleep(1)

dieGate = [-13, -23.75, 2.7]

msg = Pose()
msg.position.x = dieGate[0]
msg.position.y = dieGate[1]
msg.position.z = dieGate[2]

rospy.loginfo("publishing Die Gate")

pub.publish(msg)
msg = rospy.wait_for_message(subTopic, Empty)
rospy.sleep(1)

rouletteTable = [-17, -26.5, 2.9]

msg = Pose()
msg.position.x = rouletteTable[0]
msg.position.y = rouletteTable[1]
msg.position.z = rouletteTable[2]

rospy.loginfo("publishing rouletteTable")

pub.publish(msg)
msg = rospy.wait_for_message(subTopic, Empty)
