==============
State Machine
==============


.. image:: ./../../assets/State_Machine.png


smach_top
-----------------

This is the top level state machine.
Inside of it will be all of the sub state machines (one for each task)

* Starts ROS node :code:`state_machine`

.. class:: def main()
    
    Initialize all statemachines to be interconnected
    All state transition logic is in the manager


.. class:: def loadStateMachines(task_list)

    Loop through all tasks in the task_list
    Initialize their state machine objects
    Add the objects to sm_list and return

.. class:: def createTransitionsForManager(task_list, final_outcome)


configure_run
-----------------

sudo apt-get install qtcreator pyqt5-dev-tools

Future ToDo: add transdec support


.. class:: ClickableLabel(figure, task, clicked_func, rotation=0, widget_size=(480, 640))

.. class:: Cusub_GUI(map_name, map_config, mission_config, simulation)



Tasks
----------------


droppers
##############

    Droppers Task, attempts to drop 2 markers in the bin

    Objectives:

    - Search

    - Approach

.. class:: Droppers()


.. class:: Approach(task_name, listener, clients)

**Subscribers**

* *topic_name*, *type*
* :code:`cusub_perception/mapper/task_poses`, *Detection*

.. class:: Drop(task_name, listener, clients)

**Subscribers**

* *topic_name*, *type*
* :code:`cusub_perception/mapper/task_poses`, *Detection*

**Service Proxies**

* *service_name*, *service_class*
* :code:`cusub_common/activateActuator`, *ActivateActuator*


.. class:: Retrace(task_name, listener)





jiangshi
##############

Jiangshi Buoy Task, attempts to bump into the jiangshi buoy

Objectives:

- Search
- Approach
- Slay
- Back Up

.. class:: Jiangshi(Task)


.. class:: Approach(task_name, listener)


.. class:: Slay(task_name)


.. class:: Backup(task_name)


.. class:: Retrace(task_name, listener)


manager
##############

Manager State
Manages state transitions between tasks
Reads in the state transitions from the mission_config parameters

.. class:: Manager(task_names, mission_end_outcome='mission_success')

**Publishers**

* *topic_name*, *type*
* :code:`cusub_cortex/state_machine/task_status`, *TaskStatus*

**Services**

* *service_name*, *service_class*
* :code:`cusub_cortex/state_machine/get_next_task`, *GetNextTask*
:ref:`Message Types<message_types>`

octagon
##############

Octagon Task, attempts to center on and rise in the octagon

Visual servos with the front camera on the octagon task until it sees the coffin's in the downcam

Visual servos with the downcam to center on the coffins

Rises slowly straight upwards until it breaches in the octagon


.. class:: Octagon()

.. class:: Rise()

**Publishers**

* *topic_name*, *type*
* :code:`cusub_common/motor_controllers/pid/depth/setpoint`, *Float64*
* :code:`cusub_common/motor_controllers/pid/drive/setpoint`, *Float64*


**Subscribers**

* *topic_name*, *type*
* :code:`cusub_common/motor_controllers/pid/depth/state`, *Float64*
* :code:`cusub_common/motor_controllers/pid/drive/state`, *Float64*


**Service Proxies**

* *service_name*, *service_class*
* :code:`cusub_perception/darknet_multiplexer/get_classes`, *DarknetClasses*



path
##############

Path Marker Task, attempts to center on the path, orient itself and update next task's prior

Objectives:

- Search
- Follow
---> Center on the path marker

---> Orient with the path marker

---> Update next task's prior

.. class:: Path(path_num_str)

.. class:: Follow(path_num_str)

    Center, orient, update next task's prior


**Service Proxies**

* *service_name*, *service_class*
* :code:`cusub_cortex/state_machine/get_next_task`, *GetNextTask*



pid_client
##############

.. class:: PIDClient(objective_name, axis, root_topic=STANDARD_ROOT_TOPIC)


**Subscribers**

* *topic_name*, *type*
* :code:`cusub_common/motor_controllers/pid/state`, *Float64*

**Service Proxies**

* *service_name*, *service_class*
* :code:`/<SUB_NAME>/cusub_common/motor_controllers/<AXIS>_mux/select`, *MuxSelect*

.. note:: 
    :code:`<SUB_NAME>` comes from parameter :code:`rospy.get_param('~robotname')`

    :code:`<AXIS>` comes as input to the :code:`PIDClient` constructor


search
##############

Search Objective

.. class:: Search(task_name, listener, target_classes, prior_pose_param_str, darknet_cameras=DARKNET_CAMERAS_DEFAULT)


start_gate
##############

StartGate Task, attempts to go through the start gate.

Receives a pose of the start gate and adjusts the pose according to the smaller side of the gate in order to maximize points.

Objectives:

1) Search (based on prior)

2) Attack (goes behind gate based on arg distBehindGate)


.. class:: StartGate()

.. class:: Attack(task_name)

**Service Proxies**

* *service_name*, *service_class*
* :code:`cusub_common/toggleWaypointControl`, *ToggleControl*


startup_task
##############

Startup Task, allows the man on the competition dock time to remove the tether from the vehicle before it starts its autonomous run.
Waits briefly at the surface before diving.


.. class:: Startup()

.. class:: Dive(task_name)


**Publishers**

* *topic_name*, *type*
* :code:`cusub_common/motor_controllers/pid/depth/setpoint`, *Float64*



task
##############

Meta Classes for all Tasks and Objectives.

.. class:: Task(name)

.. class:: Objective(outcomes, objtv_name)

    Objectives are subtasks within a task

    They have:

    Pose of the sub (self.cur_pose)
    
    a waypointNavigator client (self.wayClient)

    Helper functions { go_to_pose(), get_distance() }

**Publishers**

* *topic_name*, *type*
* :code:`cusub_common/odometry/filtered`, *Odometry*
* :code:`cusub_cortex/state_machine/task_status`, *TaskStatus*


**Service Proxies**

* *service_name*, *service_class*
* :code:`cusub_perception/darknet_multiplexer/configure_active_cameras`, *DarknetCameras*
* :code:`cusub_common/toggleWaypointControl`, *ToggleControl*

.. class:: Timeout(name)

    @brief Timeout object for tasks


triangle
##############

Triangle Buoy Task. Attempts to run into a selected side of the triangle buoy.
Approaches the bouy, uses visual servoing to orbit around the buoy, runs into the selected side

.. class:: Triangle()


.. class:: Slay()

    Go to a point in front of the bouy, slay jiangshi backwards, backup



**Publishers**

* *topic_name*, *type*
* :code:`cusub_common/motor_controllers/pid/drive/setpoint`, *Float64*
* :code:`cusub_common/motor_controllers/pid/strafe/setpoint`, *Float64*
* :code:`cusub_common/motor_controllers/pid/depth/setpoint`, *Float64*


**Subscribers**

* *topic_name*, *type*
* :code:`cusub_common/motor_controllers/pid/strafe/setpoint`, *Float64*
* :code:`cusub_common/motor_controllers/pid/drive/state`, *Float64*


**Service Proxies**

* *service_name*, *service_class*
* :code:`cusub_perception/darknet_multiplexer/get_classes`, *DarknetClasses*
* :code:`cusub_common/toggleWaypointControl`, *ToggleControl*



.. _message_types:

Defined Message Types
----------------------
* GetNextTask
    * string next_task
* TaskStatus
    * std_msgs/Header header
    * string[] task_statuses



