#ifndef LOCALIZER_CLASS_SRC_LOCALIZER_CLASS_H_
#define LOCALIZER_CLASS_SRC_LOCALIZER_CLASS_H_

#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <vector>
#include <string>
#include <darknet_ros_msgs/BoundingBoxes.h>
//#include <darknet_ros_msgs/BoundingBoxes.h>

namespace localizer_ns
{
  class Localizer : public nodelet::Nodelet
  {
  public:
    virtual void onInit();
  private:
    void loadRosParams();
    void darknetCallback(const darknet_ros_msgs::BoundingBoxes& bbs);
    std::vector<std::string> class_names;
//    std::vector<pose_generator::PoseGenerator> pose_generators;
    ros::Publisher pub;
    ros::Subscriber sub;
  };
}

#endif
