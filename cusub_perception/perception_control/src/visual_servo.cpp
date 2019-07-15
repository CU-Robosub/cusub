#include <perception_control/visual_servo.h>

namespace perception_control
{
    void VisualServo::onInit()
    {
        NODELET_INFO("Visual Servo Server Starting up!");
    }
}
PLUGINLIB_EXPORT_CLASS(perception_control::VisualServo, nodelet::Nodelet);