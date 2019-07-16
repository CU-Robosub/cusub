#include <perception_control/bb_proportional.h>

namespace perception_control
{
    BBProportional::BBProportional(ros::NodeHandle& nh) : BBController(nh)
    {
        // load rosparams
        ROS_INFO("loading BB Proportional");
    }

    void BBProportional::respond(int xdiff, int ydiff)
    {
        ROS_INFO("Respondin");
    }
}