#ifndef VISUALCONTROL_H
#define VISUALCONTROL_H

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <perception_control/OrbitBuoyAction.h>
#include <actionlib/server/simple_action_server.h>
#include <waypoint_navigator/ToggleControl.h>
#include <string>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <math.h>

namespace perception_control
{
    class VisualControl : public nodelet::Nodelet
    {
    public:
        virtual void onInit();
    private:
        ~VisualControl();
        void update(const ros::TimerEvent& e);
        bool controlPids(const bool takeControl);
        bool controllingPids;
        
        void darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr bbs);
        void odomCallback(const nav_msgs::OdometryConstPtr odom);
        void driveCallback(const std_msgs::Float64ConstPtr state);
        void yawCallback(const std_msgs::Float64ConstPtr state);
        void strafeCallback(const std_msgs::Float64ConstPtr state);

        geometry_msgs::Pose subPose;
        float driveState, yawState, strafeState;

        void execute(const perception_control::OrbitBuoyGoalConstPtr goal);
        ros::Publisher drivePub, yawPub, strafePub;
        ros::ServiceClient wayToggleClient; 
        ros::Timer timer;
        ros::Subscriber darknetSub, driveSub, yawSub, strafeSub, odomSub;
    };
}

#endif // VISUALCONTROL_H