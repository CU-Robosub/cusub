#include <perception_control/bb_proportional.h>

namespace perception_control
{
    BBProportional::BBProportional(ros::NodeHandle& nh) : BBController(nh)
    {
        ROS_INFO("loading BB Proportional");
        std::string downcamNamespace = "perception_control/bb_controllers/proportional/downcam";
        nh.getParam(downcamNamespace + "/drive/max_setpoint", downcam_drive_max_setpoint);
        nh.getParam(downcamNamespace + "/drive/maxout_pixel_dist", downcam_drive_maxout_pixel_dist);
        nh.getParam(downcamNamespace + "/strafe/max_setpoint", downcam_strafe_max_setpoint);
        nh.getParam(downcamNamespace + "/strafe/maxout_pixel_dist", downcam_strafe_maxout_pixel_dist);
    }

    void BBProportional::configureByCamera(std::string camera)
    {
        ROS_INFO("Configuring camera");
        x_max_setpoint = downcam_strafe_max_setpoint;
        y_max_setpoint = downcam_drive_max_setpoint;
        x_maxout_pixel_dist = downcam_strafe_maxout_pixel_dist;
        y_maxout_pixel_dist = downcam_drive_maxout_pixel_dist;
    }

    void BBProportional::respond(int xdiff, int ydiff)
    {
        ydiff = - ydiff;
        std_msgs::Float64 x_set, y_set;
        // X movement calc
        float x_slope = x_max_setpoint / ( (float)x_maxout_pixel_dist );
        x_set.data = x_slope * xdiff;
        if (abs(x_set.data) > x_max_setpoint) // check if we've exceeded our max setpoint range
        {
            x_set.data = x_set.data > 0 ? x_max_setpoint : -x_max_setpoint;
        }
        // Y Movement calc
        float y_slope = y_max_setpoint / ( (float)y_maxout_pixel_dist );
        y_set.data = y_slope * ydiff;
        if (abs(y_set.data) > y_max_setpoint) // check if we've exceeded our max setpoint range
        {
            y_set.data = y_set.data > 0 ? y_max_setpoint : -y_max_setpoint;
        }

        // Transform points into what the current state is
        x_set.data += *x_state;
        y_set.data += *y_state;

        x_pub->publish(x_set);
        y_pub->publish(y_set);
    }
}