"""
filter poses

Purpose is to take in poses and filter them using a moving average

"""
from __future__ import division

__author__ = "Soroush Khadem"



import rospy
from geometry_msgs.msg import Pose, PoseStamped

# super class, will hold variables and funcs accessible by all children
class Filter(object):
    def __init__(self):
        self.window_length = 10

    def average_poses(self, poses, recent_index):
        filtered_poses = filter(lambda pose: pose is not None, poses)

        x_tot = sum(pose.pose.position.x for pose in filtered_poses)
        y_tot = sum(pose.pose.position.y for pose in filtered_poses)
        z_tot = sum(pose.pose.position.z for pose in filtered_poses)

        pose_len = len(filtered_poses)

        xavg = x_tot/pose_len
        yavg = y_tot/pose_len
        zavg = z_tot/pose_len

        pose = PoseStamped()
        pose.header = poses[recent_index].header
        pose.pose.orientation = poses[recent_index].pose.orientation

        pose.pose.position.x = xavg
        pose.pose.position.y = yavg
        pose.pose.position.z = zavg


        return pose



class GatePoseFilter(Filter):
    def __init__(self):
        super(GatePoseFilter, self).__init__()
        # Start with all none's
        self.pose_window = [None for i in range(0, self.window_length)]
        self.num_poses = 0


    def send_pose(self, pose):
        index = self.num_poses%self.window_length
        self.pose_window[index] = pose
        self.num_poses += 1

        average_pose = self.average_poses(self.pose_window, index)

        return average_pose


class DicePoseFilter(Filter):
    def __init__(self):
        super(DicePoseFilter, self).__init__()

        self.pose_window = [None for i in range(0, self.window_length)]
        self.num_poses = 0

    def send_pose(self, pose):
        index = self.num_poses%self.window_length
        self.pose_window[index] = [pose]
        self.num_poses += 1

        average_pose = self.average_poses(self.pose_window, index)

        return average_pose
