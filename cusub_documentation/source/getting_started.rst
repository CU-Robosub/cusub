**************************
Getting Started with CUSub
**************************
This guide assumes you have an understanding of ROS build environments and git version controlling.
some :ref:`helpful-resources` helpful resources can be found at the ned of this page.


Setting up your workspace
#########################
The CUSub software stack consists of 4 meta-packages that contain all the runtime code for running the sub in sim and on the platform.

:doc:`\cusub_sim`: Contains all of the code related to the simulated underwater environment and descriptions of the subs.

:doc:`\cusub_common`: Hosts all of the drivers, motor controllers, sensor fusion, debugging tools and basic waypoint navigation server.

:doc:`\cusub_perception`: image classification, feature extraction, and localization of objects.

:doc:`\cusub_cortex`: Top level logic for controlling the sub including the state machine, global mapper, and bringup scripts for running the entire system as a whole.


in order to setup the CUSub workspace clone the repo into the src folder of you catkin_ws
::

    cd ~/<your_catkin_ws>/src
    git clone https://github.com/CU-Robosub/cusub

The software stack depends on a few external repositories that have been included as submodules.
To initialize them:
::

    cd cusub
    git submodule init
    git submodule update

note that as the submodules are updated by the respective repo owners it may be necessary to re-update to the latest code to ensure compatibility.


Library Dependencies
####################

In order to build the sim package the protobuf C compiler is required
::

    sudo apt-get install protobuf-c compiler #Ubuntu


Building the system
###################

The preferred method of building the CUSub software environment involves using the catkin build tool rather than the catkin_make tool.
to install catkin build
::

    sudo apt install python-catkin-tools

once the catkin build tools are installed you can build each meta-package individually to ensure your environment is properly setup.
first move back to the root of your ws
::

    cd ~/<your_caktin_ws>

The recommended order is:
::

    catkin build cusub_sim

::

    catkin build cusub_common

To get started working with the sub it is not necessary to build the perception or cortex meta-packages and it is recommended that you have these two packages working before adding the perception and cortex packages.
::

    catkin build cusub_perception

::

    catkin build cusub_cortex


.. _helpful-resources:

Helpful resources
#################

`dual boot guide
<http://dailylinuxuser.com/2015/11/how-to-install-ubuntu-linux-alongside.html>`_

`Ubunutu 18.04 release
<http://releases.ubuntu.com/18.04/>`_

`ROS Install guide
<http://wiki.ros.org/melodic/Installation>`_

`ROS Tutorial
<http://wiki.ros.org/ROS/Tutorials>`_

`Gentle Introduction to ROS
<https://www.cse.sc.edu/~jokane/agitr/agitr-letter.pdf>`_

`Bash Academy
<https://guide.bash.academy/>`_
