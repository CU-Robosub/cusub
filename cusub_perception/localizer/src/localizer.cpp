/*
  The job of the localizer is to turn darknet bounding boxes into class poses.
  
  It does this by distributing the bounding boxes and image to
  pose generators (inheritors of PoseGenerator class). 
  Pose generators return a pose and its class which are sent
  to the mapper in the form of Detection msgs.
 */

#include <localizer/localizer.h>

namespace localizer_ns
{
  Localizer::~Localizer()
  {
    delete sel_mappings["zp"];
  }

  void Localizer::onInit()
  {
    cuprint("starting up");
    pose_generator::ZPlane* zp = new pose_generator::ZPlane(); // declare on heap
    sel_mappings = {
      {"zp", zp}
    };

    if (is_nodelet)
    {
      cuprint("running as nodelet");
      ros::NodeHandle& nh = getMTNodeHandle();
      sub = nh.subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &Localizer::darknetCallback, this);
      pub = nh.advertise<localizer::Detection>("cusub_perception/mapper/task_poses",1);
      loadRosParams(nh);
    }
    else
    {
      cuprint("running NOT as a nodelet");
      ros::NodeHandle nh;
      sub = nh.subscribe("cusub_perception/darknet_ros/bounding_boxes", 1, &Localizer::darknetCallback, this);
      pub = nh.advertise<localizer::Detection>("cusub_perception/mapper/task_poses",1);
      loadRosParams(nh);
    }
    detection_num = 0;
  }

  /*
    Fills mappings data structure, provides link from darknet class to 
    which pose generator to use.
   */
  void Localizer::loadRosParams(ros::NodeHandle& nh)
  {
    map<string, string> map_params;
    if( !nh.getParam("localizer/classes", map_params))
    {
      cuprint_warn("Localizer failed to locate params");
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
    Throw out all boxes that touch the edge of the image. One of our localization assumptions is full view of the task.
   */
  bool Localizer::checkBox(const darknet_ros_msgs::BoundingBox& bb)
  {
    /*
    if( bb.xmin == 0 || bb.ymin == 0 ||  // TODO logic to allow for downcam bounding box checking
        bb.xmax >= 752 || bb.ymax >= 480) // Indeed
      { return false; }
    else { return true; }
    */
    return true;
  }

  /*
    Darknet callback, uses mappings data structure to aggregate common
    darknet bounding boxes into a vector and pass it into a pose generator.
    Publishes pose and class received from pose generator as detection msg.
   */
  void Localizer::darknetCallback(const darknet_ros_msgs::BoundingBoxesPtr bbs)
  {
    // Group all bounding boxes that use same PoseGenerator into vectors
    map<pose_generator::PoseGenerator*, vector<darknet_ros_msgs::BoundingBox>> bb_map;
    for(darknet_ros_msgs::BoundingBox box : bbs->bounding_boxes)
    {
      auto generator_mapping = mappings.find(box.Class);
      if(generator_mapping == mappings.end())
      {
        cuprint_warn(std::string("No pose generator given for ") + box.Class + std::string(". Add to localizer/config/localizer_config.yaml"));
      } else {
        if(bb_map.find(generator_mapping->second) == bb_map.end()) // pose gen ptr hasn't been added to bb_map yet
        {
          vector<darknet_ros_msgs::BoundingBox> new_bb_vector;
          bb_map[generator_mapping->second] = new_bb_vector;
        }
        if ( checkBox(box) ) { bb_map[generator_mapping->second].push_back(box); }
      }
    }

    // Call generatePose on all PoseGenerators passing in vector of bounding boxes
    for(auto generator_bbs_pair : bb_map)
    {
      vector<localizer::Detection> detections;
      bool ret = generator_bbs_pair.first->generatePose(bbs->image, generator_bbs_pair.second, detections);
      if(ret)
      {
        for(auto detection : detections){
          pub.publish(detection);
        }
      }
    }
  }
  void Localizer::cuprint(std::string str)
    {
      std::string print_str = std::string("[\033[92mLocalizer\033[0m] ");
      ROS_INFO( (print_str + str).c_str());
    }

    void Localizer::cuprint_warn(std::string str)
    {
      std::string print_str = std::string("[\033[92mLocalizer\033[0m] ");
      print_str = print_str + std::string("\033[93m[WARN] ") + str + std::string("\033[0m");
      ROS_INFO( print_str.c_str());
    }
    void Localizer::set_not_nodelet(void)
    {
      is_nodelet = false;
    }
}

PLUGINLIB_EXPORT_CLASS(localizer_ns::Localizer, nodelet::Nodelet);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "localizer_node");
  localizer_ns::Localizer l;
  l.set_not_nodelet();
  l.onInit();
  ros::spin();
}