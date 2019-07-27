#ifndef GET_CLASSES_CLASS_SRC_GET_CLASSES_CLASS_H_
#define GET_CLASSES_CLASS_SRC_GET_CLASSES_CLASS_H_

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <vector>
#include <darknet_multiplexer/DarknetClasses.h>
#include <iostream>

namespace darknet_get_classes_ns
{
    class GetClasses : public nodelet::Nodelet
    {
    public:
        virtual void onInit();
    private:
        void darknetCallback(const darknet_ros_msgs::BoundingBoxesPtr bbs);
        bool handle(darknet_multiplexer::DarknetClasses::Request& request, darknet_multiplexer::DarknetClasses::Response& response);
        std::vector<std::string> classes;
        ros::ServiceServer service;
        ros::Subscriber darknetSub;
        bool recording;
    };
}

#endif