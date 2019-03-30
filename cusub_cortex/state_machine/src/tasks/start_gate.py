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

    def __init__(self):
        super(StartGate, self).__init__(self.outcomes) # become a state machine first
        self.initObjectives()
        self.linkObjectives()

    def initObjectives(self):
        prior = self.priorList2Pose(rospy.get_param('tasks/start_gate/prior'))
        search_alg = rospy.get_param("tasks/start_gate/search_alg")
        self.search = Search(search_alg, prior, "cusub_cortex/mapper_out/start_gate")
        self.attack = Attack()

    def linkObjectives(self):
        with self: # we are a StateMachine
            smach.StateMachine.add('Search', self.search, transitions={'aborted':'Attack', 'success':'Search'})
            smach.StateMachine.add('Attack', self.attack, transitions={'success':'task_success', 'aborted':'Attack'})

class Attack(Objective):
    """
    Tell the sub to go through the gate
    """

    replanThreshold = 1.0 # if the change in startgate pose is greater than this value, we should replan our path there
    outcomes=['success','aborted']

    def __init__(self):
        rospy.loginfo("---Attack objective initializing")
        super(Attack, self).__init__(self.outcomes, "Attack")
        self.distBehind = float(rospy.get_param('tasks/start_gate/dist_behind_gate', 1.0))
        self.first_pose_received = False
        self.start_gate_pose = None
        rospy.Subscriber("cusub_cortex/mapper_out/start_gate", PoseStamped, self.start_gate_callback)

    def start_gate_callback(self, msg):
        # Set the first pose, note this case is different b/c we won't abort the Attack Objective
        if self.start_gate_pose == None:
            self.start_gate_pose = msg
            return
        
        changeInPose = self.getDistance(self.start_gate_pose.position, msg.position)
        
        if changeInPose > self.replanThreshold:
            self.start_gate_pose = new_pose
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
    #TODO namespace
    sis = smach_ros.IntrospectionServer('sis', sg.sm, '/sis')
    sis.start()
    sg.execute(0)
    sis.stop()
