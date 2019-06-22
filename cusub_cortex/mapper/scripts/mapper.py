#!/usr/bin/env python
from __future__ import division
"""
Transforms relative poses from localizer to odom frame
Stores and averages the transforms poses for continuous publishing
Publishes these averaged poses on cusub_cortex/mapper_out/ namespace
Configurable via mapper/config/mapper.yaml
"""
import rospy
import tf
from geometry_msgs.msg import Pose, PoseStamped
from localizer.msg import Detection

class ExpWeightedAvg():

    num_poses = 0

    def __init__(self, num_poses_to_avg):
        self.beta = (1.0 - float(num_poses_to_avg)) / ( - float(num_poses_to_avg))
        self.avg_pose = Pose()

    def add_new_pose(self, new_pose):
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
    
    def get_avg(self)
        return self.avg_pose

# NOTE landmark code has been moved to landmark.py file for future integration

class Mapper(object):

    classes = {}

    def __init__(self):
        self.pose_sub = rospy.Subscriber('cusub_perception/mapper/task_poses', Detection, self.pose_received)
        self.listener = tf.TransformListener()
        self.load_objects()
        self.sub_name = rospy.get_param('~sub_name')
        self.timer = rospy.Timer(rospy.Duration(1 / rospy.get_param('mapper/publish_freq')), self.publish_poses)

    def load_objects(self):
        classes_dict = rospy.get_param('mapper/classes')
        for class_name in classes_dict:
            self.classes[class_name] = {}
            self.classes[class_name]['pub'] = rospy.Publisher('cusub_cortex/mapper_out/'+class_name, PoseStamped, queue_size = 10)
            self.classes[class_name]['filter'] = ExpWeightedAvg(rospy.get_param('mapper/classes/'+class_name))
            self.classes[class_name]['unfiltered_pub'] = rospy.Publisher('cusub_cortex/mapper_out/unfiltered/'+class_name, PoseStamped, queue_size = 10)
            self.classes[class_name]['latest_pose'] = None

    def pose_received(self, msg):
        if msg.class_id in self.classes.keys():
            odom_pose = self.transform_to_odom(msg)
            if odom_pose != None:
                self.classes[class_name]['latest_pose'] = msg
                self.classes[class_name]['filter'].add_new_pose(msg.pose)
        else:
            rospy.logwarn("Mapper doesn't know about class: " + msg.class_id + ". Add to mapper/config/mapper.yaml")

    def publish_poses(self, msg):
        for class_name in self.classes:
            if self.classes[class_name]['latest_pose'] != None:
                msg = PoseStamped()
                msg.header = self.classes[class_name]['latest_pose'].header
                msg.pose = self.classes[class_name]['filter'].get_avg()
                self.classes[class_name]['pub'].publish(msg)
                self.classes[class_name]['unfiltered_pub'].publish(self.classes[class_name]['latest_pose'])
    
    def transform_to_odom(self, cv_pose):
        try:
            self.listener.waitForTransform('/'+ self.sub_name + '/description/odom', cv_pose.header.frame_id, cv_pose.header.stamp, rospy.Duration(0.2))
            pose = self.listener.transformPose('/' + self.sub_name + '/description/odom', cv_pose)
        except (tf.ExtrapolationException, tf.ConnectivityException, tf.LookupException) as e:
            rospy.logwarn(e)
            pose = None
        return pose

def main():
    rospy.init_node('mapper')
    m = Mapper()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == '__main__':
    main()
