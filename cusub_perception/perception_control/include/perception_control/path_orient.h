#ifndef PATH_ORIENT_CLASS_SRC_PATH_ORIENT_CLASS_H_
#define PATH_ORIENT_CLASS_SRC_PATH_ORIENT_CLASS_H_

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <perception_control/PathOrientAction.h>
#include <actionlib/server/simple_action_server.h>
#include <waypoint_navigator/ToggleControl.h>
#include <std_msgs/Float64.h>

namespace perception_control
{
    // typedef actionlib::SimpleActionServer<perception_control::OrbitBuoyAction> orbitServer;

    class PathOrient : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
        // private:
        //     void darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs);
        //     float yawState;
    };
}

#endif