************
CUSub Common
************

This section will go into the details of the common parts of the CU robosub.

Running Common
##############

In order to run the common packages it is recomend to launch:
::

    roslaunch cusub_common leviathan_sim.launch

keep in mind that this package must be launched on top of the sim launches. this means that both a gazebo environment and the leviathan description must be launched.


System Packages
###############
.. toctree::
    :maxdepth: 1
    :hidden:

    cusub_common_bringup
    waypoint_navigator
    debugging_tools
    motor_controllers


cusub_common: meta-package for organizing software stack

:doc:`cusub_common_bringup <cusub_common_bringup>`

:doc:`Waypoint Navigator <waypoint_navigator>`

:doc:`Debugging tools <debugging_tools>`

:doc:`Motor controllers <motor_controllers>`


Drivers
#######
.. toctree::
	:maxdepth: 1
    :hidden:

    drivers/gazebo_drivers
    drivers/actuator
    drivers/depth_sensor
    drivers/dvl
    drivers/occam
    drivers/pololu_controller
    drivers/sparton_imu


.. autoclass:: Console
    :members:

.. autoclass:: JoyTeleop
    :members:

:doc:`gazebo_drivers <drivers/gazebo_drivers>`

:doc:`actuator <drivers/actuator>`

:doc:`Depth sensor <drivers/depth_sensor>`

:doc:`occam <drivers/occam>`

:doc:`pololu_controller <drivers/pololu_controller>`

:doc:`sparton imu <drivers/sparton_imu>`


ROS Topic Interface
###################

Currently the majority of topics will be placed under this namespace:
::

    <sub_name>/cusub_common/<topics>

the exceptions being motor_controllers and occam due to their large topic list:
::

    <sub_name>/cusub_common/motor_controllers/<topics>
    <sub_name>/cusub_common/occam/<topics>


Known Issues
############

will provide known issues and solutions for this meta-package
