*****************
cusub_sim_bringup
*****************

This package hosts the launch files for the simulator elements

Environment Launchers
#####################

**divewell_startgate.launch**: launches the divewell environment with the 2018 startgate.

**divewell_all_task.launch**: launches the divewell environment with all the 2018 startgate, 2018 dice buoys, two 2018 path markers, and the 2018 Roulette table.

**TRANSDEC_21.launch**: launches the TRANSDEC environment with the 2018 competition elements; startgate, dice buoys, path markers, Roulette table.

**herkules.launch**: launches the environment containing the Herkules shipwreck in Trondheim Norway

Sub Launchers
#############

**leviathan_description**: Loads the Leviathan description into the the gazebo environment

**triton description**: Loads the Triton description into the gazebo environment

.. code-block:: xml

    passable arguments:
    <arg name="x" default="-7.5"/>
    <arg name="y" default="1"/>
    <arg name="z" default="-1"/>
    <arg name="yaw" default="0"/>
    <arg name="namespace" default="<sub>/description" />



