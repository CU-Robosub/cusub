#ifndef BB_CONTROLLER_CLASS_SRC_BB_CONTROLLER_CLASS_H_
#define BB_CONTROLLER_CLASS_SRC_BB_CONTROLLER_CLASS_H_

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace perception_control
{
    class BBController
    {
    public:
        virtual void respondDowncamDrive(float diff) {;}
        virtual void respondDowncamStrafe(float diff) {;}
        virtual void respondDowncamYaw(float diff) {;}
        virtual void respondDowncamDepth(float diff) {;}

        virtual void respondOccamDrive(float diff) {;}
        virtual void respondOccamStrafe(float diff) {;}
        virtual void respondOccamYaw(float diff) {;}
        virtual void respondOccamDepth(float diff) {;}

        virtual float getTargetYaw(const std::string &occamFrame) {;}
        virtual void targetYaw(float &target) {;}

        BBController(ros::NodeHandle& nh);
    protected:
        float driveState, strafeState, yawState, depthState;
        ros::Publisher drivePub, strafePub, yawPub, depthPub;
    private:
        void driveCallback(const std_msgs::Float64ConstPtr state);
        void strafeCallback(const std_msgs::Float64ConstPtr state);
        void yawCallback(const std_msgs::Float64ConstPtr state);
        void depthCallback(const std_msgs::Float64ConstPtr state);

        ros::Subscriber driveSub, strafeSub, yawSub, depthSub;
    };
}

#endif