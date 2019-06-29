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
from geometry_msgs.msg import Pose, PoseStamped, Quaternion
from localizer.msg import Detection
import numpy as np

MAX_SUM_TASK_POSITION_VALUE=500

class ExpWeightedAvg():

    num_poses = 0

    def __init__(self, num_poses_to_avg):
        self.num_poses_to_avg = num_poses_to_avg
        self.beta = (1.0 - float(num_poses_to_avg)) / ( - float(num_poses_to_avg))
        self.avg_pose = Pose()
        self.pose_list_x = []
        self.pose_list_y = []
        self.pose_list_z = []
        self.pose_list_yaw = []

    def add_new_pose(self, new_pose):
        if abs(new_pose.position.x) + abs(new_pose.position.y) + abs(new_pose.position.z) > MAX_SUM_TASK_POSITION_VALUE:
            rospy.logwarn_throttle(1,"Mapper rejecting outlier pose: " + str(new_pose))
            return
        elif self.num_poses == 0:
            self.avg_pose = new_pose

        # Convert to positive euler angles
        quat_list_new = [new_pose.orientation.x, new_pose.orientation.y, new_pose.orientation.z, new_pose.orientation.w]
        new_angles = self.make_pos_angles(tf.transformations.euler_from_quaternion(quat_list_new))
        quat_list_avg = [self.avg_pose.orientation.x, self.avg_pose.orientation.y, self.avg_pose.orientation.z, self.avg_pose.orientation.w]
        avg_angles = self.make_pos_angles(tf.transformations.euler_from_quaternion(quat_list_avg))

        # if self.num_poses > self.num_poses_to_avg:
        #     if ( abs(self.avg_pose.position.x - new_pose.position.x) > 3*np.std(self.pose_list_x) ) or \
        #     ( abs(self.avg_pose.position.y - new_pose.position.y) > 3*np.std(self.pose_list_y) ) or \
        #     ( abs(self.avg_pose.position.z - new_pose.position.z) > 3*np.std(self.pose_list_z) ): #or \
        #     # ( abs(avg_angles[2] - new_angles[2]) > 3*np.std(self.pose_list_yaw) ):
        #         rospy.logwarn_throttle(1,"Mapper rejecting outlier pose:\n" + str(new_pose))
        #         return
        #     else:
        #         self.pose_list_x.pop(0)
        #         self.pose_list_y.pop(0)
        #         self.pose_list_z.pop(0)
        #         self.pose_list_yaw.pop(0)

        # self.pose_list_x.append(new_pose.position.x)
        # self.pose_list_y.append(new_pose.position.y)
        # self.pose_list_z.append(new_pose.position.z)
        # self.pose_list_yaw.append(new_angles[2])

        self.num_poses += 1

        x_avg = self.avg_pose.position.x
        y_avg = self.avg_pose.position.y
        z_avg = self.avg_pose.position.z

        # Average positions and orientations
        new_x_avg = (self.beta * x_avg) + (1-self.beta) * (new_pose.position.x)
        new_y_avg = (self.beta * y_avg) + (1-self.beta) * (new_pose.position.y)
        new_z_avg = (self.beta * z_avg) + (1-self.beta) * (new_pose.position.z)
        new_roll_avg = (self.beta * avg_angles[0]) + (1-self.beta) * new_angles[0]
        new_pitch_avg = (self.beta * avg_angles[1]) + (1-self.beta) * new_angles[1]
        new_yaw_avg = (self.beta * avg_angles[2]) + (1-self.beta) * new_angles[2]

        
        # Store avg orientation as quaternion
        new_quat_list_avg = tf.transformations.quaternion_from_euler(0, 0, new_yaw_avg) # assume no pitch or roll
        q = Quaternion()
        q.x = new_quat_list_avg[0]
        q.y = new_quat_list_avg[1]
        q.z = new_quat_list_avg[2]
        q.w = new_quat_list_avg[3]

        # Store avg new pose
        pose = Pose()
        pose.position.x = new_x_avg
        pose.position.y = new_y_avg
        pose.position.z = new_z_avg
        # pose.orientation = q
        pose.orientation = q
        self.avg_pose = pose

    def make_pos_angles(self, angles):
        """ Make all angles 0 - 2pi for convenient averaging """
        pos_angles = []
        for angle in angles:
            angle = (angle + 2*np.pi) % (2*np.pi)
            pos_angles.append(angle)
        return pos_angles

    def get_avg(self):
        return self.avg_pose

# NOTE landmark code has been moved to landmark.py file for future integration

class Mapper(object):

    classes = {}

    def __init__(self):
        self.listener = tf.TransformListener()
        self.load_objects()
        self.sub_name = rospy.get_param('~sub_name')
        self.timer = rospy.Timer(rospy.Duration(1 / rospy.get_param('mapper/publish_freq')), self.publish_poses)
        self.pose_sub = rospy.Subscriber('cusub_perception/mapper/task_poses', Detection, self.pose_received)

    def load_objects(self):
        classes_dict = rospy.get_param('mapper/classes')
        for class_name in classes_dict:
            self.classes[class_name] = {}
            self.classes[class_name]['pub'] = rospy.Publisher('cusub_cortex/mapper_out/'+class_name, PoseStamped, queue_size = 10)
            self.classes[class_name]['filter'] = ExpWeightedAvg(rospy.get_param('mapper/classes/'+class_name))
            self.classes[class_name]['unfiltered_pub'] = rospy.Publisher('cusub_cortex/mapper_out/unfiltered/'+class_name, PoseStamped, queue_size = 10)
            self.classes[class_name]['latest_pose_stamped'] = None

    def pose_received(self, msg):
        if msg.class_id in self.classes.keys():
            odom_pose = self.transform_to_odom(msg.pose)
            if odom_pose != None:
                self.classes[msg.class_id]['latest_pose_stamped'] = odom_pose
                self.classes[msg.class_id]['filter'].add_new_pose(odom_pose.pose)
                
        else:
            rospy.logwarn("Mapper doesn't know about class: " + msg.class_id + ". Add to mapper/config/mapper.yaml")

    def publish_poses(self, msg):
        for class_name in self.classes:
            if self.classes[class_name]['latest_pose_stamped'] != None:
                msg = PoseStamped()
                msg.header = self.classes[class_name]['latest_pose_stamped'].header
                msg.pose = self.classes[class_name]['filter'].get_avg()
                self.classes[class_name]['pub'].publish(msg)
                self.classes[class_name]['unfiltered_pub'].publish(self.classes[class_name]['latest_pose_stamped'])
    
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
