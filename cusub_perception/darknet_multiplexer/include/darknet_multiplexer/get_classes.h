#ifndef GET_CLASSES_CLASS_SRC_GET_CLASSES_CLASS_H_
#define GET_CLASSES_CLASS_SRC_GET_CLASSES_CLASS_H_

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <vector>

namespace darknet_get_classes_ns
{
    class GetClasses : public nodelet::Nodelet
    {
    public:
        virtual void onInit();
    private:
        void darknetCallback(const darknet_ros_msgs::BoundingBoxesPtr bbs);
        std::vector<std::string> classes;
    };
}

#endif