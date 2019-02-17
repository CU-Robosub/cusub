#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

from sensor_msgs.msg import JointState

from gazebo_msgs.srv import SpawnModel

class GoldChipSpawner():

    current_dropper = 0

    def __init__(self):
        pass

    def jointFeetback(self, fb):
        if fb.position[0] < -0.03:
            self.dispenseChip()

    def dispenseChip(self):

        if(self.current_dropper < 1):

            rospy.loginfo("Simulating droppers by spawning one.")

            spawner = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
            result = spawner(

                # model_name
                ("gc_dropper_%d" % (self.current_dropper)),

                # Get the dropper URDF from param
                # model_xml
                rospy.get_param("~dropperURDF"),

                # robo_namespace'
                ("gc_dropper_%d" % (self.current_dropper)),

                # Spawn dropper just below sub
                # inital_pose
                Pose(Point(0.14,-0.085,-0.265), Quaternion(0.0,0.0,0.0,1.0)),

                # spawn in baselink so position is relative to sub
                # reference_frame
                "GoldChip_1::goldchip_button_holder_link"

                # TODO test spwaning in leviathan reference frame and ditch ground truth
            )

            if not result.success:
                rospy.logerr("Failed to spawn dropper!")
                rospy.logerr(result.status_message)

            self.current_dropper = self.current_dropper + 1

        else:
            rospy.logerr("You already got your chips!")

    def run(self):

        self.button_joint_sub = rospy.Subscriber("/GoldChip_1/joint_states", JointState, self.jointFeetback)

        rospy.spin()

if __name__ == "__main__":
    rospy.init_node('GoldChipSpawner')
    a = GoldChipSpawner()
    try:
        a.run()
    except rospy.ROSInterruptException:
      rospy.logerr("Gold Chip Spawner has died!");
