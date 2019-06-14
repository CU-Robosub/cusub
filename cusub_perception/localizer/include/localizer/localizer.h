#ifndef LOCALIZER_CLASS_SRC_LOCALIZER_CLASS_H_
#define LOCALIZER_CLASS_SRC_LOCALIZER_CLASS_H_

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <vector>
#include <map>
#include <string>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <localizer/pose_generator.h>
#include <localizer/start_gate_watershed.h>
#include <geometry_msgs/Pose.h>

namespace localizer_ns
{
  class Localizer : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
  private:
    void loadRosParams(ros::NodeHandle& nh);
    void darknetCallback(const darknet_ros_msgs::BoundingBoxesPtr bbs);
    std::map<std::string, pose_generator::PoseGenerator*> mappings;
    ros::Publisher pub;
    ros::Subscriber sub;
  };

  namespace pose_gen_decls
  {
    // Pose Generators Declarations
    pose_generator::StartGateWatershed sgw;
  }
  // Pose Generator Mappings
  std::map<std::string, pose_generator::PoseGenerator*> sel_mappings =  {
    { "watershed", &pose_gen_decls::sgw},
    {"bouy_pnp", &pose_gen_decls::sgw}
  };
  
}

#endif
