==============
Depth Sensor
==============


.. class:: Depth_sensor

    This class reads data sent over serial from arduino and publishes a depth
    Zeros the depth sensor by sending information to arduino over serial

* Starts ROS node :code:`Depth_Pub`

**Subscribers**

* *topic_name*, *type*
* :code:`zero_command`, *Empty*

**Publishers**

* :code:`depth_odom`, *PoseWithCovarianceStamped*
* :code:`depth_map`, *PoseWithCovarianceStamped*
* :code:`/kill_switch`, *Bool*



