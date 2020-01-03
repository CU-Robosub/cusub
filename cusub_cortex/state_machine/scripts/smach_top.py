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
from waypoint_navigator.srv import ToggleControl
from waypoint_navigator.msg import waypointAction, waypointGoal
import actionlib
from tasks.jiangshi_det import Jiangshi
from tasks.start_gate import StartGate
# from tasks.bangbang_dice_task import BangBangDiceTask
# from tasks.bangbang_roulette_task import BangBangRouletteTask
# from tasks.naive_visual_servo_objective import NaiveVisualServoTask
# from tasks.dropper_task import DropperTask
# from tasks.bangbang_roulette_task import BangBangRouletteTask
# from tasks.jiangshi import Jiangshi
from tasks.manager import Manager
from tasks.task import Timeout
from tasks.path import Path
from tasks.dropper import Dropper
from tasks.startup_task import Startup
from tasks.triangle import Triangle
from tasks.octagon import Octagon
# from tasks.debug_servo import Debug

class bcolors: # For terminal colors
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'

def smprint(string, warn=False):
    if warn:
        rospy.loginfo("[" + bcolors.OKGREEN + "Startup Script" + bcolors.ENDC + "] " + bcolors.WARNING +"[WARN] "+ string + bcolors.ENDC)
    else:
        rospy.loginfo("[" + bcolors.OKGREEN + "Startup Script" + bcolors.ENDC + "] " + string)

def createTransitionsForManager(task_list, final_outcome):
    transition_dict = {}
    for task in task_list:
        transition_dict[task] = task
    transition_dict[final_outcome] = final_outcome
    return transition_dict


def loadStateMachines(task_list):
    """
    Loop through all tasks in the task_list
    Initialize their state machine objects
    Add the objects to sm_list and return
    """
    sm_list = []
    for task in task_list:
        task_sm = None
        smprint("Loading " + bcolors.HEADER + task.capitalize() + bcolors.ENDC + " state machine")

        if task == "start_gate":
            task_sm = StartGate()
        elif task == "jiangshi":
            task_sm = Jiangshi()
        elif task == "path0":
            task_sm = Path("0")
        elif task == "path1":
            task_sm = Path("1")
        elif task == "dropper":
            task_sm = Dropper()
        elif task == "startup":
            task_sm = Startup()
        elif task == "triangle":
            task_sm = Triangle()
        elif task == "octagon":
            task_sm = Octagon()
        elif task == "debug":
            task_sm = Debug()
        else:
            raise ValueError("Unrecognized task: {}".format(task))

        # Check for proper initialization of the task statemachine
        if task_sm == None:
            raise Exception("You omitted initializing the task! task_sm = Task()")

        sm_list.append( task_sm )

    return sm_list

def main():
    """
    Initialize all statemachines to be interconnected
    All state transition logic is in the manager
    """
    rospy.init_node('state_machine')
    
    # All Objectives depend on the waypoint server so let's wait for it to initalize here
    wayClient = actionlib.SimpleActionClient('/'+rospy.get_param('~robotname')+'/cusub_common/waypoint', waypointAction)
    smprint("waiting for waypoint server")
    # wayClient.wait_for_server()
    smprint("...connected")

    # Make sure the waypoint navigator is in control
    #toggle_waypoint = rospy.ServiceProxy('cusub_common/toggleWaypointControl', ToggleControl)
    #toggle_waypoint(True)

    if rospy.get_param('~using_darknet'):
        smprint("waiting for darknet multiplexer server")
 #       rospy.wait_for_service("cusub_perception/darknet_multiplexer/configure_active_cameras")
        smprint("...connected")
    else:
        smprint("SM NOT using darknet configuration service.", warn=True)

    # initialize all of the state machines
    smprint("waiting for rosparams")
    while not rospy.has_param("mission_tasks") and not rospy.is_shutdown():
        rospy.sleep(1)
    smprint("...found")

    final_outcome = "mission_completed"
    sm_top = smach.StateMachine(outcomes=[final_outcome])
    sm_top.userdata.previous_outcome = "starting"
    sm_top.userdata.timeout_obj = Timeout()

    task_list = rospy.get_param("mission_tasks")
    manager = Manager(task_list, final_outcome)
    sm_list = loadStateMachines(task_list)

    # Load all statemachines
    with sm_top:
        manager_transitions = createTransitionsForManager(task_list, final_outcome)
        smach.StateMachine.add('manager', manager, \
            transitions=manager_transitions, \
            remapping={"timeout_obj":"timeout_obj", "previous_outcome":"previous_outcome"})
        for index in range(len(task_list)):
            sm_name = task_list[ index ]
            sm = sm_list[ index ]
            smach.StateMachine.add(sm_name, sm, \
                transitions={'manager':'manager'},\
                remapping={'outcome':'previous_outcome'}) # all states will transition to the manager) # all states will transition to the manager
    
        outcome = sm_top.execute()

if __name__ == '__main__':
    main()
