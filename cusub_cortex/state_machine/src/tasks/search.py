"""
Searching Algorithms

Approach a prior for a task, once we get a few yolo hits on searchTopic quit out

halfhalf

"""
from abc import ABCMeta, abstractmethod
import smach
from geometry_msgs.msg import Point, PoseStamped, Pose
import rospy
from tasks.task import Objective
from geometry_msgs.msg import PoseStamped

POSE_INDEX = 0
WAIT_INDEX = 1

class Search(Objective):
    """ Search state that implements an indicated search algorithm """
    
    outcomes = ['found','not_found'] # We found task or timed out

    def __init__(self, priorPose, searchTopic, numQuitPoses=5):
        """
        Search objective initialization function

        Parameters
        ---------
        priorPose : Pose
             Prior pose of the task
        searchTopic : str
             Topic name to listen for task poses
        numQuitPoses : int
             Number of poses to receive before aborting and transitioning
        """

        self.prior = priorPose
        rospy.Subscriber(searchTopic, PoseStamped, self.exit_callback)
        self.numQuitPoses = numQuitPoses
        self.numPosesReceived = 0

        super(Search, self).__init__(self.outcomes, "Search")

    def exit_callback(self, msg): # Abort on the first publishing
        self.numPosesReceived += 1
        if not self.abort_requested() and self.numPosesReceived > self.numQuitPoses:
            self.request_abort()

    def execute(self, ueserdata):
        rospy.loginfo("---Executing Search")
        if self.abort_requested():
            return "found"

        if self.go_to_pose(self.prior):
            return "found"
        else:
            return "not_found"
