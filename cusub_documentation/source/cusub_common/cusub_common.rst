************
CUSub Common
************

This section will go into the details of the common parts of the CU robosub.

Running Common
##############

this section will include launch details for the meta-package


System Packages
###############
.. toctree::
    :maxdepth: 1
    :hidden:

    cusub_common_bringup
    waypoint_navigator


cusub_common: meta-package for organizing software stack

:doc:`cusub_common_bringup <cusub_common_bringup>`:

:doc:`Waypoint Navigator <waypoint_navigator>`:

Debugging Tools
___________________
.. toctree::
    :hidden:

    debugging-tools/leviathan_control

Console: unknown

leviathan_control: teleop?

qt_console: unknown

teleop: teleop

Drivers
_______

empty stub for now

Motor Controllers
_________________

.. toctree::
    :hidden:

    motor-controllers/bangbang
    motor-controllers/pid_controller

:doc:`bangbang <motor-controllers/bangbang>`:

:doc:`pid_controller <motor-controllers/pid_controller>`:



ROS Topic Interface
###################

Will contain a list of topics typically created by this meta-package, the namespace conventions and basic uses.

Known Issues
############

will provide known issues and solutions for this meta-package