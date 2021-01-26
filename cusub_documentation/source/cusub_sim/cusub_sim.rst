*********
CUSub Sim
*********
This section will go into the details of the gazebo based robosub simulator.

Running the sim
###############

Once you are able to build the system without errors you can launch the sim environment by 
first sourcing your env, then launching the Gazebo world. 
::

	source ~/<your_robosub_ws>/devel/setup.bash
	roslaunch cusub_sim_bringup divewell_startgate.launch startgate:=true

Keep in mind that this ONLY launches the simulator environment and objects. to launch the sub:
::

    roslaunch cusub_sim_bringup leviathan_description.launch

please note this does not launch the control packages or state estimation packages require to run the sub. For more information on controlling the sub check the cusub_common package.


Running a task in the sim
#########################
If you want to run a task you have to do a few more things after you launch the sim.


----------------------------
Deciding the Task
----------------------------
Deciding the task to change which task leviathan does you can uncomment the desired task at the bottom of the file:
:code:`cusub/cusub_cortex/state_machine/config/mission_config.yaml`

----------------------------
Moving objects in Gazebo
----------------------------
When doing a specific task you may want to move different objects around in the sim.
We have a GUI that lets you do this easily.

Use the following terminal commands to launch the GUI:

.. code-block::

    cd ~/<your_robosub_ws>/src/cusub/cusub_cortex/state_machine/scripts
    python configure_run.py -s t


Here you can move around all the different objects in the sim and when you are done
click the button update mission config at the bottom.

Next you need to actually move the objects to where you configured them to go.
To do this use the following terminal commands:

.. code-block::
    
    cd ~/<your_robosub_ws>/src/cusub/cusub_cortex/state_machine/scripts
    python gazebo.py -n 0


Now everything should be moved within the sim.


----------------------------
Launching the tasks
----------------------------

To start a task you need to launch the following files.
**Remember each launch file needs its own terminal and each terminal needs to be sourced**

.. code-block::
    
    roslaunch cusub_common_bringup leviathan_sim.launch
    roslaunch cusub_perception_bringup perception_sim.launch
    roslaunch state_machine state_machine.launch __ns:=leviathan


System Packages
###############
.. toctree::
    :hidden:

    cusub_sim_bringup
    leviathan_description
    robosub_descriptions
    uuv_simulator <https://uuvsimulator.github.io/>

cusub_sim: meta-package for organizing software stack

:doc:`CUSub Simulator Bringup <cusub_sim_bringup>`: contains the launch files to start the gazebo environment.

:doc:`Leviathan/Triton Descriptions <leviathan_description>`: all the files containing Leviathan and Triton's models and descriptions for gazebo.

:doc:`Robosub Descriptions <robosub_descriptions>`: contains all of the world assets for the gazebo sim including, TRANSDEC, the divewell, and obstacles.

.. TODO add uuv to intersphinx? or just hyperlink?

`uuv_simulator <https://uuvsimulator.github.io/>`_: The UUV Simulator is a package containing the implementation of Gazebo plugins and ROS nodes necessary for the simulation of unmanned underwater vehicles, such as ROVs (remotely operated vehicles) and AUVs (autonomous underwater vehicles).

ROS Topic Interface
###################

to access the gazebo world environment's topics, services, parameters, and tf frames use the namespace:
::

    <sub_name>/gazebo/<topics>

to access the the sub's model/description topics, services, parameters and tf frames use the namespace:
::

    <sub_name>/description/<topics>

Known Issues
############

the protobuf-compiler error: make sure you install the protobuf-compiler if you get a build error.
::

    sudo apt install protobuf-compiler protobuf-c-compiler
    cd ~/<your_robosub_ws>
    catkin clean
    catkin build
