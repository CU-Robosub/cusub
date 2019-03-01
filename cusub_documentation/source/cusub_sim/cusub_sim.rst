*********
CUSub Sim
*********
This section will go into the details of the gazebo based robosub simulator.

Running the sim
###############

Once you are able to build the system without errors you can launch the sim environment.
::
	
	source ~/<your_robosub_ws>/devel/setup.bash
	roslaunch cusub_sim_bringup divewell_startgate.launch

Keep in mind that this ONLY launches the simulator environment and objects. to launch the sub:
::

    roslaunch cusub_sim_bringup leviathan_description.launch

please note this does not launch the control packages or state estimation packages require to run the sub. For more information on controlling the sub check the cusub_common package.

System Packages
###############
.. toctree::
    :hidden:

    cusub_sim_bringup
    leviathan_description
    triton_description
    robosub_descriptions
    uuv_simulator <https://uuvsimulator.github.io/>

cusub_sim: meta-package for organizing software stack

:doc:`CUSub Simulator Bringup <cusub_sim_bringup>`: contains the launch files to start the gazebo environment

:doc:`Leviathan Description <leviathan_description>`: all the files containing the Leviathan's model and descriptions for gazebo.

:doc:`Triton Description <triton_description>`: all the files containing the Triton's model and descriptions for gazebo.

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