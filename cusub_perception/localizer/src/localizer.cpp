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

  void Localizer::darknetCallback(const darknet_ros_msgs::BoundingBoxesPtr bbs)
  {
    NODELET_INFO("Received darknet Image.");

    // Group all bounding boxes that use same PoseGenerator into vectors
    std::map<pose_generator::PoseGenerator*, std::vector<darknet_ros_msgs::BoundingBox>> bb_map;
    std::map<std::string, pose_generator::PoseGenerator*>::iterator it;

    NODELET_INFO("#bbs: %lu", bbs->bounding_boxes.size());

    for(darknet_ros_msgs::BoundingBox box : bbs->bounding_boxes)
    {
      it = mappings.find(box.Class);
      if(it == mappings.end())
      {
        NODELET_ERROR("No pose generator given for %s", box.Class.c_str());
      } else {
        if(bb_map.find(it->second) == bb_map.end()) // pose gen ptr hasn't been added to bb_map yet
        {
          NODELET_INFO("Creating new vector for: %s", it->first.c_str());
          std::vector<darknet_ros_msgs::BoundingBox> new_bb_vector;
          bb_map[it->second] = new_bb_vector;
        }
        NODELET_INFO("Adding %s", box.Class.c_str());
        bb_map[it->second].push_back(box);
      }
    }

    // Call generatePose on all PoseGenerators passing in vector of bounding boxes
    std::map<pose_generator::PoseGenerator*, std::vector<darknet_ros_msgs::BoundingBox>>::iterator bb_map_it;
    for(bb_map_it=bb_map.begin(); bb_map_it != bb_map.end(); bb_map_it++)
    {
      geometry_msgs::Pose p;
      std::string class_name;
      if( bb_map_it->first->generatePose(bbs->image, bb_map_it->second, p, class_name))
      {
        NODELET_INFO("Pose Localized!");
      } 
    }
  }
}

PLUGINLIB_EXPORT_CLASS(localizer_ns::Localizer, nodelet::Nodelet);