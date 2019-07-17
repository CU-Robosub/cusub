#ifndef BB_CONTROLLER_CLASS_SRC_BB_CONTROLLER_CLASS_H_
#define BB_CONTROLLER_CLASS_SRC_BB_CONTROLLER_CLASS_H_

#include <ros/ros.h>
#include <std_msgs/Float64.h>

namespace perception_control
{
    typedef enum AxisConfig_t
    {
        DRIVE_AXIS=0,
        STRAFE_AXIS=1,
        YAW_AXIS=2,
        DEPTH_AXIS=3
    }AxisConfig;

    class BBController
    {
    public:
        void configureAxes(AxisConfig x, AxisConfig y);
        virtual void respond(int xdiff, int ydiff) {;}
        BBController(ros::NodeHandle& nh);
        float* x_state,* y_state;
        ros::Publisher* x_pub,* y_pub;
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