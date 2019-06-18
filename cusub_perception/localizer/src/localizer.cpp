/*
  The job of the localizer is to turn darknet bounding boxes into class poses.
  
  It does this by distributing the bounding boxes and image to
  pose generators (inheritors of PoseGenerator class). 
  Pose generators return a pose and its class which are sent
  to the mapper in the form of Detection msgs.
 */

#include <pluginlib/class_list_macros.h>
#include <localizer/localizer.h>

namespace localizer_ns
{
  void Localizer::onInit()
  {
    ros::NodeHandle& nh = getMTNodeHandle();
    sub = nh.subscribe("/leviathan/darknet_ros/bounding_boxes", 1, &Localizer::darknetCallback, this);
    pub = nh.advertise<localizer::Detection>("Global_State/task_poses",1);
    NODELET_INFO("Starting Localizer");
    loadRosParams(nh);
  }

  /*
    Fills mappings data structure, provides link from darknet class to 
    which pose generator to use.
   */
  void Localizer::loadRosParams(ros::NodeHandle& nh)
  {
    map<string, string> map_params;
    if( !nh.getParam("localizer", map_params))
    {
      NODELET_ERROR("Localizer failed to locate params");
      abort();
    }
    map<string, pose_generator::PoseGenerator*>::iterator sel_it;
    map<string, string>::iterator it;
    for ( it=map_params.begin(); it != map_params.end(); it++)
    {
      pose_generator::PoseGenerator* pg_ptr = sel_mappings.find(it->second)->second;
      mappings[it->first] = pg_ptr;
    }
  }

  /*
    Darknet callback, uses mappings data structure to aggregate
    darknet bounding boxes into a vector and pass it into a pose generator.
    Publishes pose and class received from pose generator as detection msg.
   */
  void Localizer::darknetCallback(const darknet_ros_msgs::BoundingBoxesPtr bbs)
  {
    // Group all bounding boxes that use same PoseGenerator into vectors
    map<pose_generator::PoseGenerator*, vector<darknet_ros_msgs::BoundingBox>> bb_map;
    map<string, pose_generator::PoseGenerator*>::iterator it;
    for(darknet_ros_msgs::BoundingBox box : bbs->bounding_boxes)
    {
      it = mappings.find(box.Class);
      if(it == mappings.end())
      {
        NODELET_ERROR("No pose generator given for %s", box.Class.c_str());
      } else {
        if(bb_map.find(it->second) == bb_map.end()) // pose gen ptr hasn't been added to bb_map yet
        {
          vector<darknet_ros_msgs::BoundingBox> new_bb_vector;
          bb_map[it->second] = new_bb_vector;
        }
        bb_map[it->second].push_back(box);
      }
    }

    // Call generatePose on all PoseGenerators passing in vector of bounding boxes
    map<pose_generator::PoseGenerator*, vector<darknet_ros_msgs::BoundingBox>>::iterator bb_map_it;
    for(bb_map_it=bb_map.begin(); bb_map_it != bb_map.end(); bb_map_it++)
    {
      localizer::Detection det;
      det.pose.header = bbs->image.header;
      if( bb_map_it->first->generatePose(bbs->image, bb_map_it->second, det.pose.pose, det.class_id))
      {
        NODELET_INFO("Pose Localized!");
        pub.publish(det);
      } 
    }
  }
}

PLUGINLIB_EXPORT_CLASS(localizer_ns::Localizer, nodelet::Nodelet);