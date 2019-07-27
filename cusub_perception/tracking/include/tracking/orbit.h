#ifndef ORBIT_CLASS_SRC_ORBIT_CLASS_H_
#define ORBIT_CLASS_SRC_ORBIT_CLASS_H_

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <tracking/OrbitBuoyAction.h>
#include <actionlib/server/simple_action_server.h>
#include <waypoint_navigator/ToggleControl.h>
#include <string>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <math.h>

namespace perception_control
{
    typedef actionlib::SimpleActionServer<perception_control::OrbitBuoyAction> orbitServer;

    class Orbit : public nodelet::Nodelet
    {
    public:
        virtual void onInit();
    private:
        ~Orbit();
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
        orbitServer* server;

        perception_control::OrbitBuoyGoalConstPtr activeGoal;
        int numberSoloFrames;
        bool vampireFound;
    };
}

#endif