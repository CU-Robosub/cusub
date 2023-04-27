#ifndef BB_PROPORTIONAL_CLASS_SRC_BB_PROPORTIONAL_CLASS_H_
#define BB_PROPORTIONAL_CLASS_SRC_BB_PROPORTIONAL_CLASS_H_

#include <perception_control/bb_controller.h>
#include <string>
#include <std_msgs/Float64.h>

namespace perception_control
{
    class BBProportional : public BBController
    {
    public:
        void configureByCamera(std::string camera);
        void respond(int xdiff, int ydiff);
        BBProportional(ros::NodeHandle& nh);
    private:
        float x_max_setpoint, y_max_setpoint;
        int x_maxout_pixel_dist, y_maxout_pixel_dist;

        // To be eliminated with pulling from the param server
        float downcam_drive_max_setpoint, downcam_strafe_max_setpoint;
        int downcam_drive_maxout_pixel_dist, downcam_strafe_maxout_pixel_dist;
    };
}

#endif