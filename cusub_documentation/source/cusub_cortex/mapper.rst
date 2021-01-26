==============
Mapper
==============


mapper
-----------------

Transforms relative poses from localizer to odom frame

Stores and averages the transforms poses for continuous publishing

Publishes these averaged poses on :code:`cusub_cortex/mapper_out/namespace`

Configurable via mapper/config/mapper.yaml

.. class:: Mapper

**Subscribers**

* *topic_name*, *type*
* :code:`cusub_perception/mapper/task_poses`, *Detection*


**Publishers**

* *topic_name*, *type*
* :code:`cusub_cortex/mapper_out/<CLASS_NAME>`, *PoseStamped*
* :code:`cusub_cortex/mapper_out/unfiltered/<CLASS_NAME>`, *PoseStamped*

.. note::
    :code:`<CLASS_NAME>` comes from parameter server: :code:`rospy.get_param('mapper/classes')`


.. class:: ExpWeightedAvg(num_poses_to_avg)


.. class:: MovingMedianAvg(num_poses_to_avg)

    Use a moving median averaging technique to reject outliers



landmark
-----------------


Maps and landmarks for multisub map building

.. class:: MapLandmark(name, marker_id, mesh)

    Landmark object



.. class:: Map()

    Map builder object, provides methods to add and update landmarks.


**Publishers**

* *topic_name*, *type*
* :code:`cusub_cortex/mapper/map_markers`, *MarkerArray*


prior_landmark
-----------------

This script pulls priors from our config yaml and creates markers for Rviz to plot

* Starts ROS node :code:`prior_landmark_markers`


.. class:: PriorLandmarker()

    Map builder object, provides methods to add and update landmarks.


**Publishers**

* *topic_name*, *type*
* :code:`cusub_cortex/mapper/prior_markers`, *MarkerArray*



odom_faker
-----------------

* Starts ROS node :code:`odom_faker`

.. class:: main()

    Sometimes Gazebo doesn't publish the ground truth pose (triangular_buoy)
    This class publishes odom msgs to mimick gazebo's GT


.. note:: Just a function, no class
    

**Publishers**

* *topic_name*, *type*
* :code:`/triangular_buoy_1/pose_gt`, *Odometry*


sim_truth_pub
-----------------

* Starts ROS node :code:`sim_truth_pub`

.. class:: SimTruthPub()

    Publishes gazebo's truth data as Detection msgs to the mapper
    Configurable via the config/sim_truth.yaml


**Publishers**

* *topic_name*, *type*
* :code:`cusub_perception/mapper/task_poses`, *Detection*

**Subscribers**

* *topic_name*, *type*
* :code:`<SIM_PUB>`, *Odometry*

.. note:: 
    :code:`<SIM_PUB>` comes from parameter :code:`rospy.get_param('sim_pub/')`






