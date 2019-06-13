#include <pluginlib/class_list_macros.h>
#include <localizer/localizer.h>

namespace localizer_ns
{
  void Localizer::onInit()
  {
    ;
  }
  void Localizer::loadRosParams()
  {
    ;
  }
  void Localizer::darknetCallback(const darknet_ros_msgs::BoundingBoxes& bbs)
  {
    ;
  }
}

PLUGINLIB_EXPORT_CLASS(localizer_ns::Localizer, nodelet::Nodelet);
