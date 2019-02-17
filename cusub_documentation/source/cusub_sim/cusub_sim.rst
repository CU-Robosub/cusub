*********
CUSub Sim
*********
This section will go into the details of the gazebo based robosub simulator.

Running the sim
###############

Once you are able to build the system without errors you can launch the basic sim.
::
	
	source ~/<your_robosub_ws>/devel/setup.bash
	roslaunch robosub_scenarios divewell_startgate.launch

Keep in mind that this ONLY launches the simulator environment and objects. It does not launch the control packages or state estimation packages require to control the model. For more information check the cusub_common package.

if you have any issues or questions be sure to post within the slack or attend the next software meeting.

System Packages
###############
.. toctree::
    :hidden:

    cusub_sim_bringup
    leviathan_description
    leviathan_gazebo_drivers
    robosub_descriptions
    uuv_simulator <https://uuvsimulator.github.io/>


cusub_sim: meta-package for organizing software stack

:doc:`CUSub Simulator Bringup <cusub_sim_bringup>`: contains the launch files to start the gazebo environment

:doc:`Leviathan Description <leviathan_description>`: all the files containing the leviathan's model and descriptions for gazebo.

:doc:`Leviathan Gazebo Drivers <leviathan_gazebo_drivers>`: contains drivers for translation of Gazebo commands to rest of cusub software stack

:doc:`Robosub Descriptions <robosub_descriptions>`: contains all of the world assets for the gazebo sim including, TRANSDEC, the divewell, and Obstacles

.. TODO add uuv to intersphinx? or just hyperlink?

`uuv_simulator <https://uuvsimulator.github.io/>`_: The UUV Simulator is a package containing the implementation of Gazebo plugins and ROS nodes necessary for the simulation of unmanned underwater vehicles, such as ROVs (remotely operated vehicles) and AUVs (autonomous underwater vehicles).

ROS Topic Interface
###################

Will contain a list of topics typically created by this meta-package, the namespace conventions and basic uses.


Known Issues
############

the protobuf-compiler. make sure you install the protobuf-compiler if you get  error while building.
::

    sudo apt install protobuf-compiler protobuf-c-compiler

Do a build of your catkin workspace