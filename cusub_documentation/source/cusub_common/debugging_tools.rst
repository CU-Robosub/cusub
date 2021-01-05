================
Debugging Tools
================
stub for information about the debugging tools for the sub.

This package will be combined into one package soon.


.. contents:: Contents

---------------
Console 
---------------

.. class:: Console


* starts `tkinter <https://docs.python.org/3/library/tkinter.html>`_ GUI for logging
* starts ROS node :code:`Console`




**Subscribers**

* *topic_name*, *type*
* :code:`/local_control/pid/yaw/state`, *Float64*
* :code:`/local_control/pid/drive/state`, *Float64*
* :code:`/local_control/pid/depth/state`, *Float64*
* :code:`/local_control/pid/strafe/state`, *Float64*

**Publishers**

* *topic_name*, *type*
* :code:`/yaw_PID/parameter_updates`, *Config*
* :code:`/depth_PID/parameter_updates`, *Config*
* :code:`/drive_PID/parameter_updates`, *Config*
* :code:`/strafe_PID/parameter_updates`, *Config*



EmbeddedMonitor
---------------

* Various sub alerts


--------------------
Leviathan Control
--------------------


joy_teleop
------------

.. class:: JoyTeleop

* Allows joystick teleoperation of submarine
* starts ROS node :code:`joy_teleop`

**Subscribers**

* *topic_name*, *type*
* :code:`motor_controllers/pid/yaw/state`, *Float64*
* :code:`motor_controllers/pid/drive/state`, *Float64*
* :code:`motor_controllers/pid/strafe/state`, *Float64*

**Publishers**

* *topic_name*, *type*
* :code:`motor_controllers/pid/yaw/setpoint`, *Float64*
* :code:`motor_controllers/pid/drive/setpoint`, *Float64*
* :code:`motor_controllers/pid/strafe/setpoint`, *Float64*
* :code:`motor_controllers/mux/yaw/control_effort`, *Float64*
* :code:`motor_controllers/mux/drive/control_effort`, *Float64*
* :code:`motor_controllers/mux/strafe/control_effort`, *Float64*
* :code:`motor_controllers/pid/pitch/setpoint`, *Float64*
* :code:`motor_controllers/pid/roll/setpoint`, *Float64*
* :code:`motor_controllers/pid/depth/setpoint`, *Float64*
* :code:`/leviathan/description/outer_controller/command`, *Float64*
* :code:`/leviathan/description/inner_controller/command`, *Float64*


**Service Proxies**

* *service_name*, *service_class*
* :code:`toggleWaypointControl`, *ToggleControl*
* :code:`activateActuator`, *ActivateActuator*


key_teleop
------------

.. class:: key_teleop

* Allows keyboard teleoperation of submarine
* starts ROS node :code:`key_teleop`

**Subscribers**

* *topic_name*, *type*
* :code:`motor_controllers/pid/yaw/state`, *Float64*
* :code:`motor_controllers/pid/drive/state`, *Float64*
* :code:`motor_controllers/pid/strafe/state`, *Float64*

**Publishers**

* *topic_name*, *type*
* :code:`/leviathan/thrusters/0/input`, *FloatStamped*
* :code:`/leviathan/thrusters/1/input`, *FloatStamped*
* :code:`/leviathan/thrusters/2/input`, *FloatStamped*
* :code:`/leviathan/thrusters/3/input`, *FloatStamped*
* :code:`/leviathan/thrusters/4/input`, *FloatStamped*
* :code:`/leviathan/thrusters/5/input`, *FloatStamped*
* :code:`/leviathan/thrusters/6/input`, *FloatStamped*
* :code:`/leviathan/thrusters/7/input`, *FloatStamped*



**Service Proxies**

* *service_name*, *service_class*
* :code:`toggleWaypointControl`, *ToggleControl*
* :code:`activateActuator`, *ActivateActuator*




--------------------
teleop
--------------------

teleop
------------

.. class:: Motor_Controller

* starts ROS node :code:`Motor_Controller`

**Publishers**

* *topic_name*, *type*
* :code:`pololu_control/command`, *Float64MultiArray*


fun_teleop
------------

.. class:: Motor_Controller

* starts ROS node :code:`Motor_Controller`

**Publishers**

* *topic_name*, *type*
* :code:`pololu_control/command`, *Float64MultiArray*
* :code:`local_control/pid/yaw/setpoint`, *Float64*
* :code:`local_control/pid/roll/setpoint`, *Float64*
* :code:`local_control/pid/pitch/setpoint`, *Float64*
* :code:`local_control/pid/depth/setpoint`, *Float64*
* :code:`local_control/pid/drive/setpoint`, *Float64*
* :code:`local_control/pid/strafe/setpoint`, *Float64*



teleop_setpoints
-----------------

.. class:: Motor_Controller

* starts ROS node :code:`Motor_Controller`

**Publishers**

* *topic_name*, *type*
* :code:`pid/yaw/setpoint`, *Float64*
* :code:`pid/roll/setpoint`, *Float64*
* :code:`pid/pitch/setpoint`, *Float64*
* :code:`pid/depth/setpoint`, *Float64*
* :code:`pid/drive/setpoint`, *Float64*
* :code:`pid/strafe/setpoint`, *Float64*






