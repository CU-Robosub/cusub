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
    typedef actionlib::SimpleActionServer<perception_control::PathOrientAction> pathServer;

    class PathOrient : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
        private:
            void darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs);
            void yawCallback(const std_msgs::Float64ConstPtr state);
            bool controlPids(const bool takeControl);
            void execute(const perception_control::PathOrientGoalConstPtr goal);
            float yawState;
            ros::Publisher yawPub;
            ros::Subscriber darknetSub, yawSub;
            ros::ServiceClient wayToggleClient;
            bool controllingPids, orientedWithPath;
            pathServer* server;
    };
}

#endif