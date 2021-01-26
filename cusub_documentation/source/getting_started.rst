**************************
Getting Started with CUSub
**************************
This guide assumes you have an understanding of ROS build environments and git version controlling.
Some :ref:`helpful-resources` helpful resources can be found at the end of this page.

.. contents:: Contents


Setting up your workspace
#########################
The CUSub software stack consists of 4 meta-packages that contain all the runtime code for running the sub in sim and on the platform.

:doc:`\cusub_sim/cusub_sim`: Contains all of the code related to the simulated underwater environment and descriptions of the subs.

:doc:`\cusub_common/cusub_common`: Hosts all of the drivers, motor controllers, sensor fusion, debugging tools and basic waypoint navigation server.

:doc:`\cusub_perception`: image classification, feature extraction, and localization of objects.

:doc:`\cusub_cortex`: Top level logic for controlling the sub including the state machine, global mapper, and bringup scripts for running the entire system as a whole.

.. note:: 
    There is also a Dockerfile install.
        

In order to setup the CUSub workspace clone the repo into the src folder of you catkin_ws
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

    sudo apt install protobuf-compiler protobuf-c-compiler


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





Docker install
##################################################

.. warning::
    
    Not quite fully polished.

    Still has some issues but the core commands to make sure everything is installed to get the sub running are there.

Nvidia Docker setup
-----------------------

    



Install Docker
____________________


.. code-block:: bash

    # Update system
    $ sudo apt-get update
    $ sudo apt-get upgrade 
    # Docker Dependencies
    $ sudo apt-get install apt-transport-https ca-certificates curl software-properties-common  
    # Add docker PGP key
    $ curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -  
    # Add docker repo
    $ sudo add-apt-repository "deb [arch=amd64] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable"  
    # Install docker
    $ sudo apt-get update
    $ sudo apt-get install docker-ce  
    # Test docker
    $ docker -v
    # Docker version 19.03.12, build 48a66213fe  
    # add user to docker group so you dont need sudo when using it
    $ sudo usermod -aG docker Username  
    # Log out and back in to apply changes, or run
    $ su - Username

Install Nvidia-Docker
______________________


.. code-block:: bash

    # Add Nvidia docker key and source list
    $ curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -$ curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
    # Update
    $ sudo apt-get update  
    # Install docker and reload daemons
    $ sudo apt-get install -y nvidia-docker2
    $ sudo pkill -SIGHUP dockerd  
    # Make sure latest NVidia drivers are installed, replace number "450" with latest
    $ sudo apt-get install nvidia-450  
    # Test nvidia docker
    docker run --runtime=nvidia --rm nvidia/cuda:9.0-base nvidia-smi
    # Ouput should be similar to:
    Sun Aug  2 00:29:45 2020
    +-----------------------------------------------------------------------------+
    | NVIDIA-SMI 440.95.01    Driver Version: 440.95.01    CUDA Version: 10.2     |
    |-------------------------------+----------------------+----------------------+
    | GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
    | Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
    |===============================+======================+======================|
    |   0  GeForce RTX 2060    On   | 00000000:01:00.0  On |                  N/A |
    | N/A   49C    P8     9W /  N/A |    386MiB /  5926MiB |      2%      Default |
    +-------------------------------+----------------------+----------------------+

    +-----------------------------------------------------------------------------+
    | Processes:                                                       GPU Memory |
    |  GPU       PID   Type   Process name                             Usage      |
    |=============================================================================|
    +-----------------------------------------------------------------------------+

.. note::

    May need to reboot for this to work depending on if kernel was updated




Download the :download:`Dockerfile <../assets/ROS-Docker>`.


.. code-block:: bash

    docker build --tag=robosub:melodic-bionic ROS-Docker
    # Need to clone latest sources
    # Need to build in custom Gazebo 9.4
    sudo apt install xpra
    # Download x11docker and run
    git clone https://github.com/mviereck/x11docker
    ./x11docker --runtime=nvidia --xpra --user=root --share=/dev/input/js0 robosub:melodic-bionic xterm
    roslaunch teleop joy.launch start_joy:=true joy_device:=/dev/input/js0 setpoint:=false namespace:=leviathan/cusub_common


**References:**

* `x11docker <https://github.com/mviereck/x11docker>`_
* `Nvidia Docker <https://github.com/NVIDIA/nvidia-docker>`_
* `Create docker container with ROS <https://github.com/osrf/docker_images/blob/0b33e61b5bbed5b93b9fba2d5bae5db604ff9b58/ros/melodic/ubuntu/bionic/ros-core/Dockerfile>`_






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
