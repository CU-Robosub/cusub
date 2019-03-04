"""
Searching Algorithms

We have a prior but are not sufficiently confident on an object's location. Let's approach the prior in a manner that let's us angle our view to get a better pose estimate of the object.

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

POSE_INDEX = 0
WAIT_INDEX = 1

class Search(Objective):
    """ Search state that implements an indicated search algorithm """
    outcomes = ['success','aborted']
    def __init__(self, searchAlg, priorPose):
        rospy.loginfo("---Search objective initializing")
        assert type(priorPose) == Pose
        self._loadSearchAlg(searchAlg)
        self.prior = priorPose
        super(Search, self).__init__(self.outcomes, "Search") 
        
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
        if self.abort_requested():
            return "aborted"
        
        rospy.loginfo("Executing Search")
        path = self.search.get_path(self.curPose, self.prior)
        for pose_and_wait in path: # path includes both a pose to reach and the waiting time
            pose = pose_and_wait[POSE_INDEX]
            wait_secs = pose_and_wait[WAIT_INDEX]
            if self.goToPose(pose):
                rospy.loginfo("---"+self.name+" aborted")
                return "aborted"
            rospy.sleep(wait_secs) # wait at our destination this many seconds
        return "success" # we should get preempted before this, if not we should loop on this task until finding our task object

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
