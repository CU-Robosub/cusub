#!/usr/bin/env python
"""
The purpose of this node will be to take in the poses published by the various cv
nodes and use some type of averaging to have reference to each task's pose in
the global frame.

Eventually, this may be replaced by mapper_server in order to use actionlib

NOTE: the unfiltered poses are left in for visual debugging purposes
"""

import rospy
import tf
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from localizer.msg import Detection

from pose_filter import filter_poses

class Mapper():
    def __init__(self):

        # new
        self.pose_sub = rospy.Subscriber('/Global_State/task_poses', Detection, self.pose_received)

        # separate topics for each task obstacle
        # dice
        self.dice1_pub = rospy.Publisher('/Global_State/dice1_pose', PoseStamped, queue_size=10)
        self.dice2_pub = rospy.Publisher('/Global_State/dice2_pose', PoseStamped, queue_size=10)
        self.dice5_pub = rospy.Publisher('/Global_State/dice5_pose', PoseStamped, queue_size=10)
        self.dice6_pub = rospy.Publisher('/Global_State/dice6_pose', PoseStamped, queue_size=10)

        self.dice1_unfiltered_pub = rospy.Publisher('/unfiltered/dice1_pose', PoseStamped, queue_size=10)
        self.dice2_unfiltered_pub = rospy.Publisher('/unfiltered/dice2_pose', PoseStamped, queue_size=10)
        self.dice5_unfiltered_pub = rospy.Publisher('/unfiltered/dice5_pose', PoseStamped, queue_size=10)
        self.dice6_unfiltered_pub = rospy.Publisher('/unfiltered/dice6_pose', PoseStamped, queue_size=10)

        self.dice1_pose = PoseStamped()
        self.dice2_pose = PoseStamped()
        self.dice5_pose = PoseStamped()
        self.dice6_pose = PoseStamped()

        self.dice1_unfiltered_pose = PoseStamped()
        self.dice2_unfiltered_pose = PoseStamped()
        self.dice5_unfiltered_pose = PoseStamped()
        self.dice6_unfiltered_pose = PoseStamped()

        self.dice1_found = False
        self.dice2_found = False
        self.dice5_found = False
        self.dice6_found = False

        # start gate
        self.start_gate_pub = rospy.Publisher('/Global_State/start_gate', PoseStamped, queue_size=10)

        self.start_gate_unfiltered_pub = rospy.Publisher('/unfiltered/start_gate', PoseStamped, queue_size=10)

        self.start_gate_pose = PoseStamped()

        self.start_gate_unfiltered_pose = PoseStamped()

        self.start_gate_found = False

        # filters
        self.gate_filter = filter_poses.GatePoseFilter()
        self.dice_filter = filter_poses.DicePoseFilter()



        #TODO: this should be dynamic, and corresponding transform from occam to odom should happen
        self.dice1_unfiltered_pose.header.frame_id = 'occam0_frame'
        self.dice2_unfiltered_pose.header.frame_id = 'occam0_frame'
        self.dice5_unfiltered_pose.header.frame_id = 'occam0_frame'
        self.dice6_unfiltered_pose.header.frame_id = 'occam0_frame'

        self.start_gate_unfiltered_pose.header.frame_id = 'occam0_frame'


        # tranforms
        self.listener = tf.TransformListener()

    def pose_received(self, detection):
        pose = PoseStamped()
        pose.pose = detection.location
        pose.header.stamp = detection.image_header.stamp
        pose.header.frame_id = detection.camera_frame

        transformed_pose = self.transform_occam_to_odom(pose)

        object = detection.object_type

        # TODO: change time to detection's time
        if object == 'dice1':
            self.dice1_unfiltered_pose = transformed_pose
            self.dice1_pose = self.dice_filter.send_pose(transformed_pose)
            self.dice1_found = True

        elif object == 'dice2':
            self.dice2_unfiltered_pose = transformed_pose
            self.dice2_pose = self.dice_filter.send_pose(transformed_pose)
            self.dice2_found = True

        elif object == 'dice5':
            self.dice5_unfiltered_pose = transformed_pose
            self.dice5_pose = self.dice_filter.send_pose(transformed_pose)
            self.dice5_found = True

        elif object == 'dice6':
            self.dice6_unfiltered_pose = transformed_pose
            self.dice6_pose = self.dice_filter.send_pose(transformed_pose)
            self.dice6_found = True

        elif object == 'start_gate':
            self.start_gate_unfiltered_pose = transformed_pose
            self.start_gate_pose = self.gate_filter.send_pose(transformed_pose)
            self.start_gate_found = True



    # occam_pose is PoseStamped with frame id being the occam frame it came from
    # i.e. occam0_frame, occam1_frame, occam4_frame
    def transform_occam_to_odom(self, occam_pose):
        # below needed ?
        # now = rospy.Time(0)
        # self.listener.waitForTransform('odom', 'base_link', now, rospy.Duration(1))
        # occam_pose.header.stamp = now

        # Gotta love tf
        odom_pose = self.listener.transformPose('odom', occam_pose)
        return odom_pose


    def publish_poses(self):
        r = rospy.Rate(10) # 10 Hz

        while not rospy.is_shutdown():
            if self.dice1_found:
                self.dice1_pub.publish(self.dice1_pose)
                self.dice1_unfiltered_pub.publish(self.dice1_unfiltered_pose)

            if self.dice2_found:
                self.dice2_pub.publish(self.dice2_pose)
                self.dice2_unfiltered_pub.publish(self.dice2_unfiltered_pose)

            if self.dice5_found:
                self.dice5_pub.publish(self.dice5_pose)
                self.dice5_unfiltered_pub.publish(self.dice5_unfiltered_pose)

            if self.dice6_found:
                self.dice6_pub.publish(self.dice6_pose)
                self.dice6_unfiltered_pub.publish(self.dice6_unfiltered_pose)


            if self.start_gate_found:
                self.start_gate_pub.publish(self.start_gate_pose)
                self.start_gate_unfiltered_pub.publish(self.start_gate_unfiltered_pose)

            r.sleep()

def main():
    rospy.init_node('mapper')
    m = Mapper()
    try:
        m.publish_poses()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
