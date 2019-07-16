#ifndef BB_CONTROLLER_CLASS_SRC_BB_CONTROLLER_CLASS_H_
#define BB_CONTROLLER_CLASS_SRC_BB_CONTROLLER_CLASS_H_

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace perception_control
{
    class BBController
    {
    public:
        virtual void respond(int xdiff, int ydiff) {;}
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