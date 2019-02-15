#!/usr/bin/env python

"""
StartGate Task, attempts to go through the start gate
Objectives:
1) Search (based on prior)
2) Attack (goes behind gate based on arg distBehindGate)

"""
from tasks.task import Task, Objective
from tasks.search import Search
import rospy
import smach
import smach_ros
from geometry_msgs.msg import PoseStamped, Pose, Point
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Empty


class StartGate(Task):

    outcomes = ['task_success','task_aborted']

    def __init__(self, gatePosePrior, searchAlg, distBehindGate):
        assert type(searchAlg) == str
        assert type(gatePosePrior) == Pose

        super(StartGate, self).__init__(self.outcomes) # become a state machine first

        self.gatePose = gatePosePrior
        self.initObjectives(gatePosePrior, searchAlg, distBehindGate)
        self.initMapperSubs()
        self.linkObjectives()

    def initObjectives(self, gatePriorPose, searchAlg, distBehind):
        self.search = Search(searchAlg, gatePriorPose)
        self.attack = Attack(distBehind)

    def initMapperSubs(self):
        rospy.Subscriber('/Global_State/start_gate', PoseStamped, self.start_gate_pose_cb)

    def linkObjectives(self):
        with self: # we are a StateMachine

            # NOTE we should be aborted before we reach the pose, aborting means we've found what we're looking for before we finished our path
            smach.StateMachine.add('Search', self.search, transitions={'aborted':'Attack', 'success':'Search'})


            smach.StateMachine.add('Attack', self.attack, transitions={'success':'task_success', 'aborted':'Attack'})

    def start_gate_pose_cb(self, msg):
        self.gatePose = msg.pose
        self.attack.start_gate_pose = msg.pose

        # check if Search is the current state and if so abort it
        if not self.search.abort_requested():
            self.search.request_abort() # we've found our pose, so stop searching

class Attack(Objective):
    """
    Tell the sub to go through the gate
    """

    replanThreshold = 1.0 # if the change in startgate pose is greater than this value, we should replan our path there
    _start_gate_pose = None
    outcomes=['success','aborted']

    def __init__(self, distBehindGate):
        rospy.loginfo("---Attack objective initializing")
        self.distBehind = distBehindGate
        self.first_pose_received = False
        super(Attack, self).__init__(self.outcomes, "Attack")

    @property
    def start_gate_pose(self):
        """
        Properties allow us to determine when start_gate_pose variable has been changed (See start_gate_pose setter )
        """
        return self._start_gate_pose

    @start_gate_pose.setter
    def start_gate_pose(self, new_pose):
        """"
        We just got a new pose for the startgate, only update it if it's different enough and if so request an abort on the start gate to replan
        """
        # Set the first pose, note this case is different b/c we won't abort the Attack Objective
        if self._start_gate_pose == None:
            self._start_gate_pose = new_pose
            return
        
        changeInPose = self.getDistance(self._start_gate_pose.position, new_pose.position)
        
        if changeInPose > self.replanThreshold:
            self._start_gate_pose = new_pose
            self.request_abort() # this will loop us back to execute

    def execute(self, userdata):
        rospy.loginfo("Executing Attack")
        self.clear_abort()

        targetPose = self.start_gate_pose # let's just go to the gate...
#        targetPose = self._getPoseBehindStartGate(self.start_gate_pose, self.distBehind)

        if self.goToPose(targetPose):
            return 'aborted'
        return "success"

    def _getPoseBehindStartGate(self, gatePose, distBehind):
        """
        Get the point that is distBehind the startgate for the sub to go to
        """
        assert type(distBehind) == float
        assert type(gatePose) == Pose

        orientation_list = [ gatePose.orientation.x, \
                             gatePose.orientation.y, \
                             gatePose.orientation.z, \
                             gatePose.orientation.w ]
        (groll, gpitch, gyaw) = euler_from_quaternion(orientation_list)

        orientation_list = [ self.curPose.orientation.x, \
                             self.curPose.orientation.y, \
                             self.curPose.orientation.z, \
                             self.curPose.orientation.w ]
        (sroll, spitch, syaw) = euler_from_quaternion(orientation_list)



        # use the orientation of the gate to then place a point right behind it
        # TODO do the calculation for a gate facing any direction

        return gatePose


if __name__ == "__main__":
    rospy.init_node("start_gate_test")
    ps = Pose()
    ps.position.x = 1
    ps.position.y = 1
    ps.position.z = 1
    sg = StartGate(ps, 'simple', 1.0)
    sis = smach_ros.IntrospectionServer('fuckuluke', sg.sm, '/fuck')
    sis.start()
    sg.execute(0)
    sis.stop()
