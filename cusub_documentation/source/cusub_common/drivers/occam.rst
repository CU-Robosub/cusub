==============
Occam
==============

auto_exposure 
------------------

* Starts ROS node :code:`auto_exposure`

**Publishers**

* *topic_name*, *type*
* :code:`leviathan/set_exposure`, *Float64*

**Services**

* *service_name*, *service_class*
* :code:`activateActuator`, *ActivateActuator*

camera_info_pub
-------------------

* Starts ROS node :code:`occam_camera_info_pub`

**Publishers**

* *topic_name*, *type*
* :code:`camera_info_topic`, *CameraInfo*


camera_set_server
-------------------

* Starts ROS node :code:`occam_set_servers`

**Services**

* *service_name*, *service_class*
* :code:`/occam/image0/set_camera_info`, *SetCameraInfo*
* :code:`/occam/image1/set_camera_info`, *SetCameraInfo*
* :code:`/occam/image2/set_camera_info`, *SetCameraInfo*
* :code:`/occam/image3/set_camera_info`, *SetCameraInfo*
* :code:`/occam/image4/set_camera_info`, *SetCameraInfo*



