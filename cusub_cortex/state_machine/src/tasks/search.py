"""
Searching Algorithms

Approach a prior for a task, once we get a few yolo hits on searchTopic quit out

SEARCH TYPES:
simple | brief: go straight to the prior pose
zigzag | brief: make a zigzag motion to the prior pose
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

    def __init__(self, priorPose, searchAlg, searchTopic, numQuitPoses=5):
        """
        Search objective initialization function

        Parameters
        ---------
        priorPose : Pose
             Prior pose of the task
        searchAlg : str
             Search algorithm to navigate to the prior
             'simple' is the only currently supported
        searchTopic : str
             Topic name to listen for task poses
        numQuitPoses : int
             Number of poses to receive before aborting and transitioning
        """

        rospy.loginfo("---Search objective initializing")
        self._loadSearchAlg(searchAlg)
        self.prior = priorPose
        rospy.Subscriber(searchTopic, PoseStamped, self.exit_callback)
        self.numQuitPoses = numQuitPoses
        self.numPosesReceived = 0

        super(Search, self).__init__(self.outcomes, "Search")

    def exit_callback(self, msg): # Abort on the first publishing
        self.numPosesReceived += 1
        if not self.abort_requested() and self.numPosesReceived > self.numQuitPoses:
            self.request_abort()

    def _loadSearchAlg(self, searchAlg):

        if searchAlg.lower() == 'simple':
            rospy.loginfo("---simple search")
            self.search = SimpleSearch()
        elif searchAlg.lower() == 'zigzag':
            rospy.loginfo("---zigzag search")
            self.search = ZigZagSearch()
        elif searchAlg.lower() == 'halfhalf':
            rospy.loginfo("---halfhalf search")
            self.search = HalfHalfSearch()
        else:
            raise(Exception("Unrecognized search algorithm"))

    def execute(self, ueserdata):
        rospy.loginfo("---Executing Search")
        if self.abort_requested():
            return "found"

        path = self.search.get_path(self.curPose, self.prior)
        for pose_and_wait in path: # path includes both a pose to reach and the waiting time
            pose = pose_and_wait[POSE_INDEX]
            wait_secs = pose_and_wait[WAIT_INDEX]
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'leviathan/description/map'
            pose_stamped.pose = pose
            if self.goToPose(pose_stamped):
                rospy.loginfo("---Exiting Search Obj, pose found")
                return "found"
            rospy.sleep(wait_secs) # wait at our destination this many seconds
        return "not_found" # we should get preempted before this, if not we should loop on this task until finding our task object

class SearchAlg(object):
    __metaclass__ = ABCMeta

    @abstractmethod
    def get_path(self, cur_pose, target_pose):
        """
        Returns a list of goal poses along with wait times upon reaching each pose in the form
        [ [ pose1, waitSecs1 ] , [ pose2, waitSecs2 ], [ pose3, waitSecs3 ], [ pose4, waitSecs4 ] ]
        """
        pass


class SimpleSearch(SearchAlg):
    """
    Goes straight to the target pose
    Returns the target_pose with some waiting time
    NOTE: If [None, self.wait_time] will wait in place
    """
    wait_time = 3
    def get_path(self, cur_pose, target_pose):
        return [ [ target_pose, self.wait_time ] ]

class ZigZagSearch(SearchAlg):
    """
    Does a zigzag pattern upon approaching the goal
    """
    def __init__(self):
        pass
    def get_path(self, cur_pose, target_pose): # TODO
        pass

class HalfHalfSearch(SearchAlg):
    """
    1) Goes 1/2 the distance to the target_pose
    2) Waits 2s
    3) Goes 3/4 the distance
    4) Waits 2s
    5) Reaches the point
    6) Reverses back to reach cur_pose (ideally we stop searching before this step)
    """
    def __init__(self):
        pass
    def get_path(self, cur_pose, target_pose): # TODO
        pass
