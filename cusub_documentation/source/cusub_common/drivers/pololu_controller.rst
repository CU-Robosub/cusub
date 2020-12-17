==============
Pololu Controller
==============


.. class:: PololuSerial

    for initializing and controlling the pololu over serial


* Starts ROS node :code:`Pololu_Controller`

**Subscribers**

* *topic_name*, *type*
* :code:`/drivers/pololu_control/command`, *Float64MultiArray*

controller
============

* starts :code:`PololuSerial`, spins
* starts ROS node :code:`pololu_controller`



Defined Message Types
----------------------
* MotorCommand
    * string joint_name
    * float64 position
    * float32 speed
    * float32 acceleration



