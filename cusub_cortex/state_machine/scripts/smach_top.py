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
from tasks.dice import Dice
from tasks.start_gate import StartGate
from tasks.visit_task import VisitTask
from tasks.bangbang_dice_task import BangBangDiceTask
from tasks.bangbang_roulette_task import BangBangRouletteTask
from tasks.naive_visual_servo_objective import NaiveVisualServoTask
# from tasks.dropper_task import DropperTask
# from tasks.bangbang_roulette_task import BangBangRouletteTask
from tasks.jiangshi import Jiangshi
from tasks.triangle_buoy import Triangle_Buoy


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

        if visit == True:
            task_sm = VisitTask(task)
        elif task == "start_gate":
            task_sm = StartGate()
        elif task == "dice":
            task_sm = Dice()
        elif task == "bangbang_dice":
            task_sm = BangBangDiceTask()
        elif task == "bangbang_roulette":
            task_sm = BangBangRouletteTask()
        # elif task == "dropper":
            # task_sm = DropperTask(prior, search_alg)
        elif task == "naive_visual_servo_objective":
            task_sm = NaiveVisualServoTask()
        elif task == "jiangshi":
            task_sm = Jiangshi()
        elif task == "triangle_buoy":
            task_sm = Triangle_Buoy()
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
    wayClient = actionlib.SimpleActionClient('/'+rospy.get_param('~robotname')+'/cusub_common/waypoint', waypointAction)
    rospy.loginfo("Waiting for waypoint server")
    wayClient.wait_for_server()
    rospy.loginfo("---connected to server")


    if rospy.get_param('~using_darknet'):
        rospy.loginfo("Waiting for darknet multiplexer server")
        rospy.wait_for_service("cusub_perception/darknet_multiplexer/configure_active_cameras")
        rospy.loginfo("---connected to server")
    else:
        rospy.logwarn("SM not using darknet configuration service.")

    sm_top = smach.StateMachine(['success', 'aborted'])

    # initialize all of the state machines
    rospy.loginfo("Waiting for rosparams")
    while not rospy.has_param("mission_tasks") and not rospy.is_shutdown():
        rospy.sleep(1)
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

            aborted_transition = success_transition
            smach.StateMachine.add(sm_name, sm, transitions={'task_aborted' : aborted_transition, 'task_success':success_transition})


    try:
        outcome = sm_top.execute() # does this allow us to receive messages in the states?
    except rospy.ROSInterruptException:
        sys.exit()

if __name__ == '__main__':
    main()
