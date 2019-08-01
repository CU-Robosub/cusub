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
        void respondDowncamDrive(float diff);
        void respondDowncamStrafe(float diff);
        void respondDowncamYaw(float diff);
        void respondDowncamDepth(float diff);

        void respondOccamDrive(float diff);
        void respondOccamStrafe(float diff);
        void respondOccamYaw(float diff);
        void respondOccamDepth(float diff);

        BBProportional(ros::NodeHandle& nh);
    private:
        float strafe_max_setpoint, drive_max_setpoint;
        int strafe_maxout_pixel_dist, drive_maxout_pixel_dist;

        // Downcam params
        float downcam_drive_max_setpoint, downcam_strafe_max_setpoint, downcam_yaw_max_setpoint, downcam_depth_max_setpoint;
        int downcam_drive_maxout_pixel_dist, downcam_strafe_maxout_pixel_dist;
        // Occam params
        float occam_drive_max_setpoint, occam_strafe_max_setpoint, occam_yaw_max_setpoint, occam_depth_max_setpoint;
        int occam_drive_maxout_pixel_dist, occam_strafe_maxout_pixel_dist, occam_yaw_maxout_pixel_dist, occam_depth_maxout_pixel_dist;
    };
}

#endif