==============
Gazebo Drivers
==============

The gazebo drivers package stores all the scripts to fake the sensor data in gazebo.

**scripts folder**: Every sensor needs its own python script to work properly in the sim and if you are using the yolo faker to avoid gpu dependent tasks, you also need yaml camera files. These yamls are stored in the config folder and it also contains the actual calibration properties from the real world. These are used so that gazebo can properly distort the simulated image.

**launch folder**: Each launch file starts the python scripts of the appropriate sensor that the sub uses. These are dependent on sub and if new sensors are added, make sure to create the new python scripts along with adding them to the launch, so that gazebo can properly incorporate all the data the sub would experience.


yolo_faker
-----------------

* Starts ROS node :code:`YOLOFaker`


This module contains the YOLOFaker node.  It generates bounding boxes
for objects in the sub simulator by taking in information about the
camera and the objects bounding points.  It listens for the ground
truth poses from gazebo and broadcasts them as transforms in the world
frame letting it transform the points in the object frame to the world
frame to be projected in the camera frame.  This lets it come up with a
bounding box in the camera image mimicing what YOLO would would produce.
This allows code development on statemachine or classical CV algorithms
without the neural net being trained or without the use of a GPU.

To use this module you will need to provide a YAML configuration file
specifying which cameras you want to use it on, their parameters, and
which objects you want to have boxed.  Objects can have multiple
classes each with there own set of bounding boxes for bounding box
generation.  This allows for a single object to generate multiple hits
like for example a start gate could have bounding points for each of
its poles allowing them to be recognized individualy.  For an example
of the yaml file look at fakeyolo_default.yaml

To get the poses of the objects you want to track you will need to attach
pose sensors to them in gazebo.

If you are using distortion in gazebo it is important to note that due
to cropping of the barrel distortion your points will not be projected
correctly unless you adjust the cameras field of view bigger than it
actually is and then provide a camera matrix with the correct parameters
which gazebo will not publish.



.. class:: YOLOObjectClass

    Detection class object

    Holds object information for a bounding box class (i.e. the pole
    part of the startgate).  This is for parts of objects that need
    to be classified rather than classifying the object as a whole.


.. class:: YOLOObject

    Holds object information required for bounding box generation

.. class:: Camera

    Holds camera information required for bounding box generation


.. class:: YOLOFaker

    YOLO Faker Node

    Takes in configuration yaml for camera, robot, and obstacle
    configurations and generates bounding boxes


**Publishers**

* *topic_name*, *type*
* :code:`bounding_boxes_topic`, *BoundingBoxes*
* :code:`debug_image_topic`, *Image*
* :code:`multiplexer_topic_debug`, *Image*


**Subscribers**

* *topic_name*, *type*
* :code:`sub_pose_topic`, *Odometry*
* :code:`camera_info_topic`, *CameraInfo*
* :code:`image_topic`, *Image*
* :code:`multiplexer_topic`, *Image*
* :code:`odom_topic`, *Odometry*

ActuatorService
-------------------

.. class:: Actuator

    Actuator service for simulator


* Starts ROS node :code:`actuator_activate`

**Services**

* *service_name*, *service_class*
* :code:`activateActuator`, *ActivateActuator*


**Service Proxies**

* *service_name*, *service_class*
* :code:`/gazebo/spawn_urdf_model`, *SpawnModel*
* :code:`/gazebo/delete_model`, *DeleteModel*
* :code:`/gazebo/set_model_state`, *SetModelState*

CameraInfoPublisher
-------------------

.. class:: CameraInfoPublisher

    Grabs :code:`~camera_info_topic` and :code:`~frameid` from parameter server
    and publishes them.


* Starts ROS node :code:`CameraInfoPublisher`

**Service Proxies**

* *service_name*, *service_class*
* :code:`~camera_info_topic`, *CameraInfo*


hydrophones_faker
-------------------

.. class:: HydroFaker

    Simulates hydrophones in Gazebo

    should publish measurements periodically

    if sub is facing w/in Psi degrees of direction of obstacle

    - meas noise ~N(0, sigma^2)

    else

    - meas noise ~N(u, sigma^2), with some offset u ~ U(-45,45)

    - offset is preserved until sub moves position_delta meters away from that position

    - or sub turns yaw_delta meters away from where the offset was calculated

    this simulates the random fluctuations of the pinger signal at each position in transdec

    - only one object at a time can be the source of the measurements

    - available objects on: cusub_common/hydrophones/debug_pinger_options

    - current object on: cusub_common/hydrophones/active

    - configured via service: cusub_common/hydrophones/switch_active_pinger

    - check whether the noise is biased: cusub_common/hydrophones/debug_is_biased

    - a pose msg for viewing in rviz is publish on: cusub_common/hydrophones/debug_measurements



* Starts ROS node :code:`fakehydro`

**Services**

* *service_name*, *service_class*
* :code:`hydrophones/switch_active_pinger`, *PingerSwitch*

**Publishers**

* *topic_name*, *type*
* :code:`hydrophones/active`, *String*
* :code:`hydrophones/debug_is_biased`, *Bool*
* :code:`hydrophones/debug_pinger_options`, *PingerOptions*
* :code:`hydrophones/debug_measurements`, *PoseStamped*
* :code:`hydrophones/measurements`, *Hydrophones*


**Subscribers**

* *topic_name*, *type*
* :code:`/leviathan/description/pose_gt`, *Odometry*
* :code:`fakehydro/objects/odom_topic`, *Odometry*


depth_sensor
-------------------

.. class:: DepthSensor

    This node republishes of the depth sensor fluid pressure
    from gazebo into a depth pose for sensor fusion



* Starts ROS node :code:`depth_sensor_gazebo`

**Publishers**

* *topic_name*, *type*
* :code:`cusub_common/depth_odom`, *PoseWithCovarianceStamped*
* :code:`cusub_common/depth_map`, *PoseWithCovarianceStamped*


**Subscribers**

* *topic_name*, *type*
* :code:`description/pressure`, *FluidPressure*


dvl
-------------------

.. class:: DVLRemap

    Remaps and transforms the DVL data from gazebo to be like the real sub


* Starts ROS node :code:`dvl_gazebo`

**Publishers**

* *topic_name*, *type*
* :code:`cusub_common/dvl`, *TwistWithCovarianceStamped*


**Subscribers**

* *topic_name*, *type*
* :code:`description/dvl_twist`, *TwistWithCovarianceStamped*


motor_control
-------------------

.. class:: MotorControl

    This module takes motor pwm commands and mapps them to thrust commands
    for gazebo

* Starts ROS node :code:`gazebo_motor_control`


**Publishers**

* *topic_name*, *type*
* :code:`description/thrusters/<0-8>`, *FloatStamped*


**Subscribers**

* *topic_name*, *type*
* :code:`cusub_common/motor_controllers/pololu_control/command`, *Float64MultiArray*


z_odom_repub
-------------------

.. class:: odom_callback


* Starts ROS node :code:`new_odom_pub`


**Publishers**

* *topic_name*, *type*
* :code:`sensor_fusion/odometry/filtered_z_flip`, *Odometry*


**Subscribers**

* *topic_name*, *type*
* :code:`sensor_fusion/odometry/filtered`, *Odometry*



thrust_odom
-------------------

.. class:: ThrustOdom

    Publishes thrust odometry to stabalize EKF


* Starts ROS node :code:`thrust_odom_gazebo`


**Publishers**

* *topic_name*, *type*
* :code:`cusub_common/thrust_odom`, *TwistWithCovarianceStamped*


**Subscribers**

* *topic_name*, *type*
* :code:`cusub_common/motor_controllers/mux/drive/control_effort`, *Float64*
* :code:`cusub_common/motor_controllers/mux/strafe/control_effort`, *Float64*



imu_repub
-------------------

.. class:: IMURepub


* Starts ROS node :code:`imu_repub`


**Publishers**

* *topic_name*, *type*
* :code:`cusub_common/imu`, *Imu*


**Subscribers**

* *topic_name*, *type*
* :code:`description/imu`, *Imu*
* :code:`mag_offset`, *Float64*


Defined Message Types
----------------------
* ActivateActuator
    * int8 actuatorNumber
    * int32 activationTime
* Hydrophones
    * std_msgs/Header header
    * float64 azimuth
* PingerOptions
    * string[] options






