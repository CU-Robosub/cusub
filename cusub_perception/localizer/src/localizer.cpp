#include <pluginlib/class_list_macros.h>
#include <localizer/localizer.h>

namespace localizer_ns
{
  void Localizer::onInit()
  {
    ros::NodeHandle& nh = getMTNodeHandle();
    sub = nh.subscribe("/leviathan/darknet_ros/bounding_boxes", 1, &Localizer::darknetCallback, this);
    // pub = nh.advertise<>
    NODELET_INFO("Starting Localizer");
    // Sub to ros
    // init pub
    // load rosparams & create vector structures
  }
  void Localizer::loadRosParams()
  {
    ;
  }
  void Localizer::darknetCallback(const darknet_ros_msgs::BoundingBoxes& bbs)
  {
    NODELET_INFO("Received darknet Image.");
  }
}

PLUGINLIB_EXPORT_CLASS(localizer_ns::Localizer, nodelet::Nodelet);