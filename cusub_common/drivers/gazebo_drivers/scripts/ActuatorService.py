#!/usr/bin/env python

from functools import partial

from threading import Thread, Lock

import rospy

from std_msgs.msg import String

from geometry_msgs.msg import Pose, Point, Quaternion

from gazebo_msgs.msg import ModelState

from gazebo_msgs.srv import SpawnModel, SetModelState, DeleteModel
from actuator.srv import ActivateActuator

import xacro
import sys
import csv
import os, rospkg

"""
This is the simulator version of the actuator service
"""
class ActuatorService(object):

    # Keep account of how many droppers we have dropped so we dont spawn
    # 2 models with the same name
    current_dropper = 0
    current_torpedo = 0

    last_actuator_time = []

    mutex = Lock()

    def __init__(self):
        pass

    @staticmethod
    def despawn_torpedo(_, model_name):

        deleter = rospy.ServiceProxy('/gazebo/delete_model', DeleteModel)
        result = deleter(model_name)

        if not result.success:
            rospy.logerr("Failed to despawn torpedo!")
            rospy.logerr(result.status_message)
            return

    def fire_torpedo(self, side):

        rospy.loginfo("Simulating torpedo by spawning one.")

        y = -0.163068 # right side
        if(side == 'left'):
            y=  0.163068

        torpedo_name = ("torpedo_%d" % (self.current_torpedo))

        rospack = rospkg.RosPack()
        xacro_file_path = os.path.join(rospack.get_path("robosub_descriptions"), "models", "Torpedo", "Torpedo.xacro")
        xacro.substitution_args_context['arg'] = {"namespace": torpedo_name}
        doc = xacro.process_file(xacro_file_path)
        urdf = doc.toxml()

        spawner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        result = spawner(

            # model_name
            torpedo_name,

            # model_xml
            urdf, #rospy.get_param("~torpedoURDF"),

            # robo_namespace
            torpedo_name,

            # Spawn dropper just below sub
            # inital_pose
            Pose(Point(0.0, y, 0.0), Quaternion(0.0, 0.7071081, 0.0, 0.7071081)),

            # spawn in baselink so position is relative to sub
            # reference_frame
            self.robotname + "/description::" + self.robotname + "/description/base_link"

        )

        if not result.success:
            rospy.logerr("Failed to spawn torpedo!")
            rospy.logerr(result.status_message)
            return

        # Let unreal know there is a torpedo
        self.sub_simulator_spawner.publish(String("torpedo:/%s/pose_gt:0" % torpedo_name))

        # Accelerate Torpedo to kill speed
        model_state = ModelState()
        model_state.model_name = torpedo_name
        model_state.twist.linear.z = 10
        model_state.reference_frame = "torpedo_%d::%s/base_link" % (self.current_torpedo, torpedo_name)

        set_model_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        result = set_model_state(model_state)

        if not result.success:
            rospy.logerr("Failed to launch torpedo!")
            rospy.logerr(result.status_message)
            return

        despawn_torpedo_model = partial(self.despawn_torpedo,
                                        model_name=torpedo_name)
        rospy.Timer(rospy.Duration(3), despawn_torpedo_model, oneshot=True)

        self.current_torpedo = self.current_torpedo + 1

    def activate_actuator(self, req):

        self.mutex.acquire()

        pin = req.actuatorNumber        # 1-6
        time_on = req.activationTime    # ms

        now = rospy.get_rostime()

        if self.last_actuator_time[pin] < now - rospy.Duration(2.0):

            self.last_actuator_time[pin] = now

            # TODO figure out which pins are actualy the droppers and put this
            #  information into a specific ENUM stye datatype
            # TODO only allow # of droppers we would really have with a Service
            #  call to override if we need to in simulator
            if pin == 1:

                rospy.loginfo("Simulating droppers by spawning one.")

                dropper_name = ("dropper_%d" % (self.current_dropper))

                rospack = rospkg.RosPack()
                xacro_file_path = os.path.join(rospack.get_path("robosub_descriptions"), "models", "Torpedo", "Torpedo.xacro")
                xacro.substitution_args_context['arg'] = {"namespace": dropper_name}
                doc = xacro.process_file(xacro_file_path)
                urdf = doc.toxml()

                spawner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
                result = spawner(

                    # model_name
                    dropper_name,

                    # Get the dropper URDF from param
                    # model_xml
                    urdf, #rospy.get_param("~dropperURDF"),

                    # robo_namespace'
                    dropper_name,

                    # Spawn dropper just below sub
                    # inital_pose
                    Pose(Point(0.0, 0.0, -0.35), Quaternion(1.0, 0.0, 0.0, 0.0)),

                    # spawn in baselink so position is relative to sub
                    # reference_frame
                    self.robotname + "/description::" + self.robotname + "/description/base_link"

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

        self.mutex.release()
        return []

    def run(self):

        self.robotname = rospy.get_param('~robotname')

        if self.robotname == 'leviathan_2':
            self.current_dropper = 10000
            self.current_torpedo = 10000
        elif self.robotname == 'leviathan_3':
            self.current_dropper = 20000
            self.current_torpedo = 20000
        elif self.robotname == 'leviathan_4':
            self.current_dropper = 30000
            self.current_torpedo = 30000

        now = rospy.get_rostime()

        self.last_actuator_time = []
        for i in range(10):
            self.last_actuator_time.append(now)

        self.sub_simulator_spawner = rospy.Publisher("/sub_simulator/spawn", String)

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
