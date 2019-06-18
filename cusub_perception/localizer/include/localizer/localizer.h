#ifndef LOCALIZER_CLASS_SRC_LOCALIZER_CLASS_H_
#define LOCALIZER_CLASS_SRC_LOCALIZER_CLASS_H_

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <vector>
#include <map>
#include <string>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <localizer/pose_generator.h>
#include <localizer/start_gate_hough.h>
#include <localizer/ignore_pg.h>
#include <geometry_msgs/Pose.h>

using namespace std;

namespace localizer_ns
{
  class Localizer : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
  private:
    void loadRosParams(ros::NodeHandle& nh);
    void darknetCallback(const darknet_ros_msgs::BoundingBoxesPtr bbs);
    map<string, pose_generator::PoseGenerator*> mappings;
    ros::Publisher pub;
    ros::Subscriber sub;
  };

  namespace pose_gen_decls
  {
    // Pose Generators Declarations
    pose_generator::StartGateHough sgh;
    pose_generator::IgnorePG ignore_pg;
  }
  // Pose Generator Mappings
  map<string, pose_generator::PoseGenerator*> sel_mappings =  {
    { "hough", &pose_gen_decls::sgh},
    {"bouy_pnp", &pose_gen_decls::sgh},
    {"ignore", &pose_gen_decls::ignore_pg}
  };
  
}

#endif

/*  Steps to creating a New Pose Generator

1) Create header file in includes/localizer/, subclass PoseGenerator
2) Create source file in src/
3) Instaniate class above in this file in namespace pose_gen_decls
4) Add its corresponding string for the config file in sel_mappings above
5) In CMakeLists.txt, add the src file path to add_library(localizer ...)

For development you may choose to load your own jpgs or pngs from a main function in your source file.
For this you'll need to add an add_executable & target_link_libraries in the CMakeLists.txt
Make sure to cleanup and delete these dev tools before pull req or merging with develop.
 */