#!/usr/bin/env python

"""
This is the top level state machine.
Inside of it will be all of the sub state machines (one for each task)

"""

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Pose, Point
from std_msgs.msg import Empty
from waypoint_navigator.msg import waypointAction, waypointGoal
import actionlib

from tasks.start_gate import StartGate
from tasks.visit_task import VisitTask
from tasks.bangbang_dice_task import BangBangDiceTask

def genPoseMsg(list_xyz):
    pose = Pose()
    pose.position.x = list_xyz[0]
    pose.position.y = list_xyz[1]
    pose.position.z = list_xyz[2]
    return pose

def loadStateMachines(task_list):
    """
    Loop through all tasks in the task_list
    Initialize their state machine objects
    Add the objects to sm_list and return
    """
    sm_list = []
    for task in task_list:
        task_sm = None
        rospy.loginfo("Loading " + task + " state machine")

        visit = False
        # Check for just visiting a task
        if "visit" in task:
            task = task[6:] # trim "visit_"
            visit = True

        # load params common to all tasks
        prior = genPoseMsg(rospy.get_param("/tasks/" + task + "/prior"))
        search_alg = rospy.get_param("/tasks/" + task + "/search_alg")

        if visit == True:
            task_sm = VisitTask(prior, search_alg)
        elif task == "start_gate":
            dist_behind_gate = rospy.get_param("/tasks/start_gate/dist_behind_gate")
            task_sm = StartGate(prior, search_alg, dist_behind_gate)
        elif task == "bangbang_dice":
            task_sm = BangBangDiceTask(prior, search_alg)
        else:
            raise ValueError("Unrecognized task: {}".format(task))

        # Check for proper initialization of the task statemachine
        if task_sm == None:
            raise Exception("You omitted initializing the task! task_sm = Task()")

        sm_list.append( task_sm )

    return sm_list

def main():
    """
    Search rosparameter for the configuration of our state machine
    Initialize all statemachines
    Loop through the statemachines in order and link them
    """
    rospy.init_node('smach_top')

    # All Objectives depend on the waypoint server so let's wait for it to initalize here
    wayClient = actionlib.SimpleActionClient('/waypoint', waypointAction)
    rospy.loginfo("Waiting for waypoint server")
    wayClient.wait_for_server()
    rospy.loginfo("---connected to server")

    sm_top = smach.StateMachine(['success', 'aborted'])

    # initialize all of the state machines
    rospy.loginfo("Waiting for rosparams")
    while not rospy.has_param("mission_tasks") and not rospy.is_shutdown():
        pass
    rospy.loginfo("--rosparams found")

    task_list = rospy.get_param("mission_tasks")
    sm_list = loadStateMachines(task_list)

    # Trim visit from any names
    for i in range(len(task_list)):
        if "visit" in task_list[i]:
            task_list[i] = task_list[i][6:] # 6 chars to trim "visit_"

    # Load all statemachines
    with sm_top:
        last_index = len(task_list) - 1
        for index in range( len(task_list) ): # loop through all of the tasks
            sm_name = task_list[ index ]
            sm = sm_list[ index ]

            # find task success transition
            if index == last_index: # check if we're the last task
                success_transition = "success"
            else: # link to next task
                success_transition = task_list[ index + 1 ]

            # find task aborted transition
            if rospy.get_param('/tasks/'+sm_name+'/failed_repeat'):
                aborted_transition = sm_name # loop back on this task if aborted
            else:
                aborted_transition = success_transition # increment


            smach.StateMachine.add(sm_name, sm, transitions={'task_aborted' : aborted_transition, 'task_success':success_transition})


    try:
        outcome = sm_top.execute() # does this allow us to receive messages in the states?
    except rospy.ROSInterruptException:
        sys.exit()

if __name__ == '__main__':
    main()
