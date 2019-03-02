*****************
cusub_sim_bringup
*****************

Cusub_sim_bringup contains the launch packages to run the whole sim.

Environment Launchers
#####################

These launch files start by setting initial poses for all obstacles and subs. Once gazebo is running with the world that is being launched, each object is uploaded to the gazebo model at their argument locations. After the proper launch files for the subs are run, as described below, you can to drive the sub in any environment you want.

**divewell_startgate.launch**: launches the divewell environment with the 2018 startgate.

**divewell_all_task.launch**: launches the divewell environment with all the 2018 startgate, 2018 dice buoys, two 2018 path markers, and the 2018 Roulette table.

**TRANSDEC_21.launch**: launches the TRANSDEC environment with the 2018 competition elements; startgate, dice buoys, path markers, Roulette table.

**herkules.launch**: launches the environment containing the Herkules shipwreck in Trondheim Norway

Sub Launchers
#############

**leviathan_description.launch**: Loads the Leviathan description into the the gazebo environment

**triton_description.launch**: Loads the Triton description into the gazebo environment

To change starting pose and namespaces, augment these parameters in your launch files or in command line.

.. code-block:: xml

    passable arguments:
    <arg name="x" default="-7.5"/>
    <arg name="y" default="1"/>
    <arg name="z" default="-1"/>
    <arg name="yaw" default="0"/>
    <arg name="namespace" default="<sub>/description" />



