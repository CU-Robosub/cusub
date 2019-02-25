#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Pose, Point, Quaternion

from gazebo_msgs.msg import ModelState

from gazebo_msgs.srv import SpawnModel, SetModelState
from actuator.srv import ActivateActuator

"""
This is the simulator version of the actuator service
"""
class ActuatorService(object):

    # Keep account of how many droppers we have dropped so we dont spawn
    # 2 models with the same name
    current_dropper = 0

    current_torpedo = 0

    def __init__(self):
        pass

    def fire_torpedo(self, side):

        rospy.loginfo("Simulating torpedo by spawning one.")

        y = -0.163068 # right side
        if(side == 'left'):
            y=  0.163068

        spawner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        result = spawner(

            # model_name
            ("torpedo_%d" % (self.current_torpedo)),

            # Get the dropper URDF from param
            # model_xml
            rospy.get_param("~torpedoURDF"),

            # robo_namespace'
            ("torpedo_%d" % (self.current_torpedo)),

            # Spawn dropper just below sub
            # inital_pose
            Pose(Point(0.0, y, 0.0), Quaternion(0.0, 0.7071081, 0.0, 0.7071081)),

            # spawn in baselink so position is relative to sub
            # reference_frame
            "leviathan::leviathan/base_link"

        )

        if not result.success:
            rospy.logerr("Failed to spawn torpedo!")
            rospy.logerr(result.status_message)

        # Accelerate Torpedo to kill speed
        model_state = ModelState()
        model_state.model_name = "torpedo_%d" % (self.current_torpedo)
        model_state.twist.linear.z = 5
        model_state.reference_frame = "torpedo_%d::Torpedo/base_link" % (self.current_torpedo)

        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        result = set_model_state(model_state)

        if not result.success:
            rospy.logerr("Failed to launch torpedo!")
            rospy.logerr(result.status_message)

        self.current_torpedo = self.current_torpedo + 1

    def activate_actuator(self, req):

        pin = req.actuatorNumber        # 1-6
        time_on = req.activationTime    # ms

        # TODO figure out which pins are actualy the droppers and put this
        #  information into a specific ENUM stye datatype
        # TODO only allow # of droppers we would really have with a Service
        #  call to override if we need to in simulator
        if pin == 1:

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
                Pose(Point(0.0, 0.0, -0.25), Quaternion(0.0, 0.0, 0.0, 1.0)),

                # spawn in baselink so position is relative to sub
                # reference_frame
                "leviathan::leviathan/base_link"

            )

            if not result.success:
                rospy.logerr("Failed to spawn dropper!")
                rospy.logerr(result.status_message)

            self.current_dropper = self.current_dropper + 1

        elif pin == 2:

            self.fire_torpedo('left')

        elif pin == 3:

            self.fire_torpedo('right')

        else:
            rospy.logerr("This actuator is not yet simulated!")

        return []

    def run(self):

        # service to activate actuator
        rospy.Service('activateActuator', ActivateActuator, self.activate_actuator)

        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('ActuatorService')
    ACTUATOR_SERVICE = ActuatorService()
    try:
        ACTUATOR_SERVICE.run()
    except rospy.ROSInterruptException:
        rospy.logerr("Actuator Service has died!")
