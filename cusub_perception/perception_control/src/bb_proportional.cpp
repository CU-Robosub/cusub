#include <perception_control/bb_proportional.h>

namespace perception_control
{
    BBProportional::BBProportional(ros::NodeHandle& nh) : BBController(nh)
    {
        ROS_INFO("loading BB Proportional");
        std::string downcamNamespace = "perception_control/bb_controllers/proportional/downcam";
        if ( !nh.getParam(downcamNamespace + "/drive/max_setpoint", downcam_drive_max_setpoint) ) { ROS_ERROR("BBProportional couldn't locate params"); abort(); }
        nh.getParam(downcamNamespace + "/drive/maxout_pixel_dist", downcam_drive_maxout_pixel_dist);
        nh.getParam(downcamNamespace + "/strafe/max_setpoint", downcam_strafe_max_setpoint);
        nh.getParam(downcamNamespace + "/strafe/maxout_pixel_dist", downcam_strafe_maxout_pixel_dist);
    }

    void BBProportional::respond(int xdiff, int ydiff)
    {
        ROS_INFO("Respondin");
    }
}