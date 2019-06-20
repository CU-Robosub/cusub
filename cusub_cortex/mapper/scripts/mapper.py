#!/usr/bin/env python
"""
The purpose of this node will be to take in the poses published by the various cv
nodes and use some type of averaging to have reference to each task's pose in
the global frame.

Eventually, this may be replaced by mapper_server in order to use actionlib

NOTE: the unfiltered poses are left in for visual debugging purposes

There must be a cleaner way to do this than all of these if statements...
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
        self.beta = (1.0 - float(num_poses_to_avg)) / ( - float(num_poses_to_avg))
        self.avg_pose = Pose()

    def get_new_avg_pose(self, new_pose):
        self.num_poses += 1
        if self.num_poses == 1:
            self.avg_pose = new_pose

        x_avg = self.avg_pose.position.x
        y_avg = self.avg_pose.position.y
        z_avg = self.avg_pose.position.z

        new_x_avg = (self.beta * x_avg) + (1-self.beta) * (new_pose.position.x)
        new_y_avg = (self.beta * y_avg) + (1-self.beta) * (new_pose.position.y)
        new_z_avg = (self.beta * z_avg) + (1-self.beta) * (new_pose.position.z)

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
        
        self.namespace = rospy.get_param('~namespace_odom')
        
        self.listener = tf.TransformListener()
        # ns = rospy.get_namespace()
        # index = ns[1:].find('/') # find the 2nd '/'
        # sub_name = ns[0:index+2]
        # print(sub_name)
        
        # rospy.Subscriber('cusub_cortex/mapper_in/task_poses', Detection, self.pose_received)

        # separate topics for each task obstacle
        # dice
        self.dice1_pub = rospy.Publisher('cusub_cortex/mapper_out/dice1', PoseStamped, queue_size=10)
        self.dice2_pub = rospy.Publisher('cusub_cortex/mapper_out/dice2', PoseStamped, queue_size=10)
        self.dice5_pub = rospy.Publisher('cusub_cortex/mapper_out/dice5', PoseStamped, queue_size=10)
        self.dice6_pub = rospy.Publisher('cusub_cortex/mapper_out/dice6', PoseStamped, queue_size=10)

        self.dice1_unfiltered_pub = rospy.Publisher('cusub_cortex/unfiltered/dice1_pose', PoseStamped, queue_size=10)
        self.dice2_unfiltered_pub = rospy.Publisher('cusub_cortex/unfiltered/dice2_pose', PoseStamped, queue_size=10)
        self.dice5_unfiltered_pub = rospy.Publisher('cusub_cortex/unfiltered/dice5_pose', PoseStamped, queue_size=10)
        self.dice6_unfiltered_pub = rospy.Publisher('cusub_cortex/unfiltered/dice6_pose', PoseStamped, queue_size=10)

        self.roulette_unfiltered_pub = rospy.Publisher('cusub_cortex/unfiltered/roulette', PoseStamped, queue_size=10)

        self.roulette_pub = rospy.Publisher('cusub_cortex/mapper_out/roulette', PoseStamped, queue_size=10)

        self.dice1_pose = PoseStamped()
        self.dice2_pose = PoseStamped()
        self.dice5_pose = PoseStamped()
        self.dice6_pose = PoseStamped()

        self.roulette_pose = PoseStamped()

        self.dice1_unfiltered_pose = PoseStamped()
        self.dice2_unfiltered_pose = PoseStamped()
        self.dice5_unfiltered_pose = PoseStamped()
        self.dice6_unfiltered_pose = PoseStamped()

        self.dice1_found = False
        self.dice2_found = False
        self.dice5_found = False
        self.dice6_found = False

        self.roulette_found = False

        # start gate
        self.start_gate_pub = rospy.Publisher('cusub_cortex/mapper_out/start_gate', PoseStamped, queue_size=10)

        self.start_gate_unfiltered_pub = rospy.Publisher('cusub_cortex/unfiltered/start_gate', PoseStamped, queue_size=10)

        self.start_gate_pose = PoseStamped()

        self.start_gate_unfiltered_pose = PoseStamped()

        self.start_gate_found = False

        # filters
        self.gate_filter = ExpWeightedAvg(200) # filter_poses.GatePoseFilter()
        self.dice_filter = filter_poses.DicePoseFilter()

        self.dice5_filter = ExpWeightedAvg(20)
        self.dice6_filter = ExpWeightedAvg(20)

        self.roulette_filter = ExpWeightedAvg(5)

        # tranforms
        self.listener = tf.TransformListener()
        #TODO: this should be dynamic, and corresponding transform from occam to odom should happen
        self.dice1_unfiltered_pose.header.frame_id = 'occam0_frame'
        self.dice2_unfiltered_pose.header.frame_id = 'occam0_frame'
        self.dice5_unfiltered_pose.header.frame_id = 'occam0_frame'
        self.dice6_unfiltered_pose.header.frame_id = 'occam0_frame'

        self.start_gate_unfiltered_pose.header.frame_id = 'occam0_frame'
        rospy.loginfo("Mapper Initialized")

    def pose_received(self, detection):
        
        # self.listener.waitForTransform(rospy.get_namespace() + 'description/map',
        #                                rospy.get_namespace() + 'description/base_link',
        #                                detection.pose.header.stamp, rospy.Duration(1))
        # map_pose = self.transform_occam_to_map(detection.pose)
        # if map_pose is not None:
        #     self.map.update_landmark(detection.class_id, map_pose)
        
#        map_pose = self.listener.transformPose(rospy.get_namespace() + 'description/map', detection.pose)

        # rospy.loginfo("Yo received a pose dawg")
        # rospy.loginfo(detection.pose.header.frame_id)
        
        transformed_pose = self.transform_occam_to_odom(detection.pose)
        
        if transformed_pose == None:
            pass
        elif detection.class_id == 'dice1':
            self.dice1_unfiltered_pose = transformed_pose
            self.dice1_pose = transformed_pose
            self.dice1_found = True

        elif detection.class_id == 'dice2':
            self.dice2_unfiltered_pose = transformed_pose
            self.dice2_pose = transformed_pose
            self.dice2_found = True

        elif detection.class_id == 'dice5':
            self.dice5_unfiltered_pose = transformed_pose
            new_pose = PoseStamped()
            new_pose.pose = self.dice5_filter.get_new_avg_pose(transformed_pose.pose)
            new_pose.header = transformed_pose.header
            self.dice5_pose = new_pose
            self.dice5_found = True

        elif detection.class_id == 'dice6':
            self.dice6_unfiltered_pose = transformed_pose
            new_pose = PoseStamped()
            new_pose.pose = self.dice6_filter.get_new_avg_pose(transformed_pose.pose)
            new_pose.header = transformed_pose.header
            self.dice6_pose = new_pose
            self.dice6_found = True

        elif detection.class_id == 'start_gate':
            self.start_gate_unfiltered_pose = transformed_pose
            new_pose = PoseStamped()
            new_pose.pose = self.gate_filter.get_new_avg_pose(transformed_pose.pose)
            new_pose.header = transformed_pose.header
            self.start_gate_pose = new_pose
            self.start_gate_found = True

        elif detection.class_id == 'roulette':
            self.roulette_unfiltered_pose = transformed_pose
            new_pose = PoseStamped()
            new_pose.pose = self.roulette_filter.get_new_avg_pose(transformed_pose.pose)
            new_pose.header = transformed_pose.header
            self.roulette_pose = new_pose
            self.roulette_found = True

    def transform_occam_to_map(self, occam_pose):
        try:
            self.listener.waitForTransform('/'+ self.namespace + '/map', occam_pose.header.frame_id, occam_pose.header.stamp, rospy.Duration(0.2))
            pose = self.listener.transformPose('/' + self.namespace + '/map', occam_pose)
        except (tf.ExtrapolationException, tf.ConnectivityException, tf.LookupException) as e:
            rospy.logwarn(e)
            pose = None
        return pose

    def transform_occam_to_odom(self, occam_pose):
        try:
            self.listener.waitForTransform('/'+ self.namespace + '/odom', occam_pose.header.frame_id, occam_pose.header.stamp, rospy.Duration(0.2))
            pose = self.listener.transformPose('/' + self.namespace + '/odom', occam_pose)
        except (tf.ExtrapolationException, tf.ConnectivityException, tf.LookupException) as e:
            rospy.logwarn(e)
            pose = None
        return pose

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

            if self.roulette_found:
                self.roulette_pub.publish(self.roulette_pose)
                self.roulette_unfiltered_pub.publish(self.roulette_unfiltered_pose)

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
