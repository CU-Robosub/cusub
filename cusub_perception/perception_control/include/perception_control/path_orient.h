#ifndef PATH_ORIENT_CLASS_SRC_PATH_ORIENT_CLASS_H_
#define PATH_ORIENT_CLASS_SRC_PATH_ORIENT_CLASS_H_

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <perception_control/PathOrientAction.h>
#include <actionlib/server/simple_action_server.h>
#include <waypoint_navigator/ToggleControl.h>
#include <std_msgs/Float64.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/background_segm.hpp>
#include <opencv2/bgsegm.hpp>

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
            void execute(const perception_control::PathOrientGoalConstPtr goal);
            ros::NodeHandle* nh;
            float deadZone, yawCarrot;
            float yawState;
            ros::Publisher yawPub;
            ros::ServiceClient wayToggleClient;
            bool controllingPids, orientedWithPath;
            pathServer* server;
    };
}

#endif