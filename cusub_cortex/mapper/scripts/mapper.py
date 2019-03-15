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

from geometry_msgs.msg import Pose, PoseStamped
from visualization_msgs.msg import Marker, MarkerArray

from localizer.msg import Detection

from pose_filter import filter_poses

class ExpWeightedAvg():

    num_poses = 0

    def __init__(self, num_poses_to_avg):
        self.beta = (1 - num_poses_to_avg) / ( - num_poses_to_avg)
#        print "Beta: {}".format(self.beta)
        self.avg_pose = Pose()
#        print self.avg_pose

    def get_new_avg_pose(self, new_pose):
        # print "New pose: {}".format(new_pose)
        # print "Avg pose: {}".format(self.avg_pose)
        # print "Iter: {}".format(self.num_poses)
        self.num_poses += 1
        if self.num_poses == 1:
            self.avg_pose = new_pose

        x_avg = self.avg_pose.position.x
        y_avg = self.avg_pose.position.y
        z_avg = self.avg_pose.position.z

        new_x_avg = (self.beta * x_avg) + (1-self.beta) * (new_pose.position.x)
        new_y_avg = (self.beta * y_avg) + (1-self.beta) * (new_pose.position.y)
        new_z_avg = (self.beta * z_avg) + (1-self.beta) * (new_pose.position.z)

#        print self.beta ** self.num_poses
        # new_x_avg_corrected = new_x_avg / (1 - self.beta ** self.num_poses )
        # new_y_avg_corrected = new_y_avg / (1 - self.beta ** self.num_poses )
        # new_z_avg_corrected = new_z_avg / (1 - self.beta ** self.num_poses )

        pose = Pose()
        pose.position.x = new_x_avg
        pose.position.y = new_y_avg
        pose.position.z = new_z_avg
        pose.orientation = new_pose.orientation
        self.avg_pose = pose

        return self.avg_pose

class MapLandmark(object):

    MAPPED_POSES = 10

    def __init__(self, name, marker_id, mesh):

        self.name = name
        self.filter = ExpWeightedAvg(5)
        self.marker = Marker()

        self.marker.type = Marker.MESH_RESOURCE
        self.marker.mesh_resource = mesh
        self.marker.scale.x = 1
        self.marker.scale.y = 1
        self.marker.scale.z = 1

        self.marker.header.frame_id = 'world' # TODO sub map frame
        self.marker.ns = '' # todo robot_name
        self.marker.id = marker_id
        self.marker.action = Marker.ADD
        self.marker.color.a = 0.2
        self.marker.color.r = 1.0
        self.marker.color.g = 0.0
        self.marker.color.b = 0.0

    def update_pose(self, pose_stamped):

        if not self.is_mapped():
            self.filter.get_new_avg_pose(pose_stamped.pose)
            if self.is_mapped():

                marker_pose = self.filter.avg_pose

                # special case for startgate
                if self.name == "start_gate":

                    # Pose is centered at 0, fix
                    marker_pose.position.z += 0.6

                    # Has to be upright and rotated 90deg
                    quat = marker_pose.orientation
                    _, _, y = tf.transformations.euler_from_quaternion((quat.x, quat.y, quat.z, quat.w))
                    x, y, z, w = tf.transformations.quaternion_from_euler(0, 0, y + 1.57)
                    marker_pose.orientation.x = x
                    marker_pose.orientation.y = y
                    marker_pose.orientation.z = z
                    marker_pose.orientation.w = w

                self.marker.pose = marker_pose

    def is_mapped(self):
        if self.filter.num_poses > self.MAPPED_POSES:
            return True
        return False

class Map(object):

    landmarks = {}

    def __init__(self):
        self.map_markers_pub = rospy.Publisher('cusub_cortex/mapper/map_markers', MarkerArray, queue_size=10)

    def add_landmark(self, landmark):
        self.landmarks[landmark.name] = landmark

    def update_landmark(self, name, pose):
        if name in self.landmarks.keys():
            self.landmarks[name].update_pose(pose)

    def publish_mapped(self):

        markers = MarkerArray()

        for _, landmark in self.landmarks.iteritems():
            if landmark.is_mapped():
                marker = landmark.marker
                marker.header.stamp = rospy.get_rostime()
                markers.markers.append(marker)

        self.map_markers_pub.publish(markers)

class Mapper(object):

    def __init__(self):
        self.robot_name = rospy.get_namespace().split('/')[1]

        self.pose_sub = rospy.Subscriber('cusub_perception/mapper/task_poses', Detection, self.pose_received)

        dice1_landmark = MapLandmark("dice1", 1, "package://robosub_descriptions/models/Dice1/Dice1.dae")
        dice2_landmark = MapLandmark("dice2", 2, "package://robosub_descriptions/models/Dice2/Dice2.dae")
        dice5_landmark = MapLandmark("dice5", 3, "package://robosub_descriptions/models/Dice5/Dice5.dae")
        dice6_landmark = MapLandmark("dice6", 4, "package://robosub_descriptions/models/Dice6/Dice6.dae")
        start_gate_landmark = MapLandmark("start_gate", 5, "package://robosub_descriptions/models/StartGate/StartGate.dae")

        self.map = Map()
        self.map.add_landmark(dice1_landmark)
        self.map.add_landmark(dice2_landmark)
        self.map.add_landmark(dice5_landmark)
        self.map.add_landmark(dice6_landmark)
        self.map.add_landmark(start_gate_landmark)

        # separate topics for each task obstacle
        # dice
        self.dice1_pub = rospy.Publisher('cusub_cortex/mapper/dice1_pose', PoseStamped, queue_size=10)
        self.dice2_pub = rospy.Publisher('cusub_cortex/mapper/dice2_pose', PoseStamped, queue_size=10)
        self.dice5_pub = rospy.Publisher('cusub_cortex/mapper/dice5_pose', PoseStamped, queue_size=10)
        self.dice6_pub = rospy.Publisher('cusub_cortex/mapper/dice6_pose', PoseStamped, queue_size=10)

        self.dice1_unfiltered_pub = rospy.Publisher('cusub_cortex/unfiltered/dice1_pose', PoseStamped, queue_size=10)
        self.dice2_unfiltered_pub = rospy.Publisher('cusub_cortex/unfiltered/dice2_pose', PoseStamped, queue_size=10)
        self.dice5_unfiltered_pub = rospy.Publisher('cusub_cortex/unfiltered/dice5_pose', PoseStamped, queue_size=10)
        self.dice6_unfiltered_pub = rospy.Publisher('cusub_cortex/unfiltered/dice6_pose', PoseStamped, queue_size=10)

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
        self.start_gate_pub = rospy.Publisher('cusub_cortex/mapper/start_gate', PoseStamped, queue_size=10)

        self.start_gate_unfiltered_pub = rospy.Publisher('cusub_cortex/unfiltered/start_gate', PoseStamped, queue_size=10)

        self.start_gate_pose = PoseStamped()

        self.start_gate_unfiltered_pose = PoseStamped()

        self.start_gate_found = False

        # filters
        self.gate_filter = filter_poses.GatePoseFilter()
        self.dice_filter = filter_poses.DicePoseFilter()

        # tranforms
        self.listener = tf.TransformListener()

    def pose_received(self, detection):
        pose = PoseStamped()
        pose.pose = detection.location
        pose.header.stamp = detection.image_header.stamp
        pose.header.frame_id = detection.camera_frame

        obj = detection.object_type

        self.listener.waitForTransform(self.robot_name + '/description/map',
                                       self.robot_name + '/description/base_link',
                                       pose.header.stamp, rospy.Duration(1))
        map_pose = self.listener.transformPose(self.robot_name + '/description/map', pose)
        self.map.update_landmark(obj, map_pose)

        transformed_pose = self.transform_occam_to_odom(pose)

        if obj == 'dice1':
            self.dice1_unfiltered_pose = transformed_pose
            self.dice1_pose = self.dice_filter.send_pose(transformed_pose)
            self.dice1_found = True

        elif obj == 'dice2':
            self.dice2_unfiltered_pose = transformed_pose
            self.dice2_pose = self.dice_filter.send_pose(transformed_pose)
            self.dice2_found = True

        elif obj == 'dice5':
            self.dice5_unfiltered_pose = transformed_pose
            self.dice5_pose = self.dice_filter.send_pose(transformed_pose)
            self.dice5_found = True

        elif obj == 'dice6':
            self.dice6_unfiltered_pose = transformed_pose
            self.dice6_pose = self.dice_filter.send_pose(transformed_pose)
            self.dice6_found = True

        elif obj == 'start_gate':
            self.start_gate_unfiltered_pose = transformed_pose
            self.start_gate_pose = self.gate_filter.send_pose(transformed_pose)
            self.start_gate_found = True



    # occam_pose is PoseStamped with frame id being the occam frame it came from
    # i.e. occam0_frame, occam1_frame, occam4_frame
    def transform_occam_to_odom(self, occam_pose):

        # self.listener.waitForTransform('odom', 'base_link', rospy.get_rostime(), rospy.Duration(1))
        odom_pose = self.listener.transformPose(self.robot_name + '/description/odom', occam_pose)
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

            self.map.publish_mapped()

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
