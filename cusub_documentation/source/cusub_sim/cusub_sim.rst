*********
CUSub_Sim
*********

This section will go into the details of the gazebo based robosub simulator.

Building the sim
################

in your workspace clone the cusub_sim repository into <your_robosub_ws>/src/ folder
::

	git clone https://github.com/CU-Robosub/cusub_sim

Next you will have to use the git submodule to import the UUV simulator packages.
::
	
	cd cusub_sim
	git submodule init
	git submodule update

Building UUV sim requires the protobuf C complier as a dependency
::
	sudo apt-get install protobuf-c-compiler # Ubuntu


Once you have pulled both the cusub_sim repo as well as the UUV simulator submodule you will want to build the system from the root of your workspace.
::

	cd ~/<your_robosub_ws>
	catkin_make


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

cusub_sim: meta-package for organizing software stack

:doc:`leviathan_description <cusub_sim/leviathan_description>`: all the files containing the leviathan's model and descriptions for gazebo.

:doc:`leviathan_gazebo_drivers <cusub_sim/leviathan_gazebo_drivers>`: contains drivers for translation of Gazebo commands to rest of cusub software stack

:doc:`robosub_descriptions <cusub_sim/robosub_descriptions>`: contains all of the world assets for the gazebo sim including, TRANSDEC, the divewell, and Obstacles

:doc:`robosub_scenarios <cusub_sim/robosub_scenarios>`: contains the launch files to start the gazebo environment

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