==============
DVL
==============

* Starts ROS node :code:`DVL`

**Publishers**

* :code:`dvl`, *TwistWithCovarianceStamped*
* :code:`dvl/depth`, *Float64MultiArray*


**Services**

* *service_name*, *service_class*
* :code:`activateActuator`, *ActivateActuator*

Defined Message Types
----------------------
* transducer
    * float64 beam1
    * float64 beam2
    * float64 beam3
    * float64 beam4
    * std_msgs/Header header

