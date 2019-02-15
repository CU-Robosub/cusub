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

Keep in mind that this ONLY launches the simulator enviroment and objects. It does not launch the control packages or state estimation packages require to control the model. For more information check the cusub_common package.

if you have any issues or questions be sure to post within the slack or attend the next software meeting.

System Packages
###############

Will contain a list of system packages as well as links to package specific documentation. Including external site links, hand written package pages and synthesized pages based on code comments.


ROS Topic Interface
###################

Will contain a list of topics typically created by this meta-package, the namespace conventions and basic uses.


Known Issues
############

Will contain information about known issues with running this meta-package and common methods for resolving them. 
