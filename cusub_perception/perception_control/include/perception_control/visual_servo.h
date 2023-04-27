#ifndef VISUALSERVO_CLASS_SRC_VISUALSERVO_CLASS_H_
#define VISUALSERVO_CLASS_SRC_VISUALSERVO_CLASS_H_

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <actionlib/server/simple_action_server.h>
#include <waypoint_navigator/ToggleControl.h>
#include <perception_control/VisualServoAction.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <perception_control/bb_proportional.h>
#include <perception_control/bb_controller.h>
#include <std_msgs/Float64.h>

namespace perception_control
{
    typedef actionlib::SimpleActionServer<perception_control::VisualServoAction> vsServer;

    class VisualServo : public nodelet::Nodelet
    {
    public:
        virtual void onInit();
    private:
        bool controlPids(const bool takeControl);
        void execute(const perception_control::VisualServoGoalConstPtr goal);
        void darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs);

        perception_control::VisualServoGoalConstPtr activeGoal;
        std::string target_frame, target_class;
        vsServer* server;
        ros::ServiceClient wayToggleClient; 
        bool controllingPids;
        ros::Subscriber darknetSub;
        int target_pixel_x, target_pixel_y, target_pixel_threshold;
        std_msgs::Float64 frozen_x_set, frozen_y_set;
        bool frozen_controls;

        // Controllers
        BBController* current_controller;
        BBProportional* proportional_controller;
    };
}

#endif