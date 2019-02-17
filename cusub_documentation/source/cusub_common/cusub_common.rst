************
CUSub Common
************

This section will go into the details of the common parts of the CU robosub.

System Packages
###############
.. toctree::
    :hidden:

    Debugging-Console
    Drivers
    Local_Control/Local_Control
    sensor_fusion


cusub_common: meta-package for organizing software stack

:doc:`Debugging-Console <Debugging-Console>`: Contiains launch files for teleoperating the sub.

:doc:`Drivers <Drivers>`: Drivers for the hardware in the subs.

:doc:`Local Control <Local_Control/Local_Control>`: Launches control loops for the sub, point and shoot controls.

:doc:`Sensor Fusion <sensor_fusion>`: EKF for getting pose estimate from DVL, IMU, and depth sensor.
