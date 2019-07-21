#include <perception_control/path_orient.h>

namespace perception_control
{
    void PathOrient::onInit()
    {
        // subscribe to yaw
        // subscribe to darknet (try to use const pointer)
        // setup action server
        NODELET_INFO("Path Orient Server Starting Up!");
    }

    
}
PLUGINLIB_EXPORT_CLASS(perception_control::PathOrient, nodelet::Nodelet);