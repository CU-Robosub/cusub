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
    loadRosParams(nh);
  }
  
  void Localizer::loadRosParams(ros::NodeHandle& nh)
  {
    std::map<std::string, std::string> map_params;
    if( !nh.getParam("localizer", map_params))
    {
      NODELET_ERROR("Localizer failed to locate params");
      abort();
    }
    std::map<std::string, pose_generator::PoseGenerator*>::iterator sel_it;
    std::map<std::string, std::string>::iterator it;
    for ( it=map_params.begin(); it != map_params.end(); it++)
    {
      pose_generator::PoseGenerator* pg_ptr = sel_mappings.find(it->second)->second;
      mappings[it->first] = pg_ptr;
    }
  }

  void Localizer::darknetCallback(const darknet_ros_msgs::BoundingBoxes& bbs)
  {
    NODELET_INFO("Received darknet Image.");
    std::map<pose_generator::PoseGenerator*, std::vector<darknet_ros_msgs::BoundingBox>> bb_map;
  }
}

PLUGINLIB_EXPORT_CLASS(localizer_ns::Localizer, nodelet::Nodelet);