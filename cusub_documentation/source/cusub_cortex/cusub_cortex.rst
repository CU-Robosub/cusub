************
CUSub Cortex
************


Running the cortex
##################

in order to run the cortex it is recommended to launch:
::
    roslaunch cusub_cortex_bringup cortex.launch

Keep in mind that this only launches the cortex. All other meta packages must be launched by their respective bringup file.

System Packages
###############
.. toctree::
    :hidden:

    config
    global_control
    mapper
    mission_config
    state_machine

:doc:`config package <config>`

:doc:`Global Controller <global_control>`

:doc:`Mapping <mapper>`

:doc:`Mission Configuration <mission_config>`

:doc:`State Machines <state_machine>`


ROS Topic Interface
###################

All topics for cortex are placed under this namespace:
::
    <sub_name>/cusub_cortex/<topics>


Known Issues
############

no known issues at this time.
