==================
WaypointNavigator
==================

.. contents:: Contents

.. class:: WaypointNavigator


* Parent class of :code:`WaypointActionServer`
* Starts ROS node :code:`WaypointNavigator`

**Subscribers**

* *topic_name*, *type*
* :code:`motor_controllers/pid/drive/state`, *Float64*
* :code:`motor_controllers/pid/strafe/state`, *Float64*
* :code:`motor_controllers/pid/yaw/state`, *Float64*
* :code:`motor_controllers/pid/depth/state`, *Float64*


**Publishers**

* *topic_name*, *type*
* :code:`motor_controllers/pid/depth/setpoint`, *Float64*
* :code:`motor_controllers/pid/drive/setpoint`, *Float64*
* :code:`motor_controllers/pid/strafe/setpoint`, *Float64*
* :code:`motor_controllers/pid/yaw/setpoint`, *Float64*
* :code:`waypoint_controlling_pids`, *Bool*

**Services**

* *service_name*, *service_class*
* :code:`addWaypoint`
* :code:`toggleWaypointControl`
:ref:`Message Types<message_types>`


WaypointActionServer
-----------------------

.. class:: WaypointActionServer

* Launched by :code:`waypoint_navigator.launch`, :code:`leviathan_sub.launch`
* Starts ROS node :code:`waypoint_action_server`



MagnetometerCalibrator
-----------------------
.. class:: MagnetometerCalibrator
Calibrate magnetometer for uuv simulator.


**Subscribers**

* :code:`odometry/filtered`, *Odometry*
* :code:`imu`, *Imu*


**Services**

* *service_name*, *service_class*
* :code:`calibrateMagnetometer`
:ref:`Message Types<message_types>`

.. _message_types:

Defined Message Types
----------------------
* AddWaypoint
    * geometry_msgs/Point waypoint
* CalibrateMagnetometer
    * std_msgs/Float64 heading
* ToggleControl
    * bool waypoint_controlling
    * bool success
