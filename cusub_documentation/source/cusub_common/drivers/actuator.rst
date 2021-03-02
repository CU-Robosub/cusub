================
ActuatorService
================

.. class:: ActuatorService

    Interfaces with actuator over serial

* Starts ROS node :code:`ActuatorService`

**Services**

* *service_name*, *service_class*
* :code:`activateActuator`, *ActivateActuator*

actuator_activate
-------------------

.. class:: Actuator

    Activates actuator over serial


* Starts ROS node :code:`actuator_activate`

Defined Message Types
----------------------
* ActivateActuator
    * int8 actuatorNumber
    * int32 activationTime

