#!/usr/bin/env python

import serial
import time
import rospy

from nav_msgs.msg import Odometry

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from gazebo_msgs.srv import SpawnModel
from actuator.srv import ActivateActuator

"""
This is the simulator version of the actuator service
"""
class ActuatorService():

    # Keep account of how many droppers we have dropped so we dont spawn
    # 2 models with the same name
    current_dropper = 0

    def __init__(self):
        pass

    def activateActuator(self, req):

        pin = req.actuatorNumber        # 1-6
        timeOn = req.activationTime     # ms

        # TODO figure out which pins are actualy the droppers and put this
        #  information into a specific ENUM stye datatype
        # TODO only allow # of droppers we would really have with a Service
        #  call to override if we need to in simulator
        if(pin == 1):

            rospy.loginfo("Simulating droppers by spawning one.")

            spawner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            result = spawner(

                # model_name
                ("dropper_%d" % (self.current_dropper)),

                # Get the dropper URDF from param
                # model_xml
                rospy.get_param("~dropperURDF"),

                # robo_namespace'
                ("dropper_%d" % (self.current_dropper)),

                # Spawn dropper just below sub
                # inital_pose
                Pose(Point(0.0,0.0,-0.25), Quaternion(0.0,0.0,0.0,1.0)),

                # spawn in baselink so position is relative to sub
                # reference_frame
                "leviathan::leviathan/base_link"

            )

            if not result.success:
                rospy.logerr("Failed to spawn dropper!")
                rospy.logerr(result.status_message)

            self.current_dropper = self.current_dropper + 1

        else:
            rospy.logerr("This actuator is not yet simulated!")

        return []

    def run(self):

        # service to activate actuator
        s = rospy.Service('activateActuator', ActivateActuator, self.activateActuator)

        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('ActuatorService')
    a = ActuatorService()
    try:
        a.run()
    except rospy.ROSInterruptException:
      rospy.logerr("Actuator Service has died!");
