#include <darknet_multiplexer/get_classes.h>

namespace darknet_get_classes_ns
{
    void GetClasses::onInit()
    {
        NODELET_INFO("Loading Get Classes Server");
        // Set up the rosservice & subscriber to darknet
    }
}
PLUGINLIB_EXPORT_CLASS(darknet_get_classes_ns::GetClasses, nodelet::Nodelet);