#ifndef START_GATE_WATERSHED_CLASS_SRC_START_GATE_WATERSHED_CLASS_H_
#define START_GATE_WATERSHED_CLASS_SRC_START_GATE_WATERSHED_CLASS_H_

#include <nodelet/nodelet.h>
#include <localizer/pose_generator.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

namespace pose_generator
{
    class StartGateWatershed : public PoseGenerator
    {
        public:
            bool generatePose(
                sensor_msgs::Image& image, 
                std::vector<darknet_ros_msgs::BoundingBox>& bbs,
                geometry_msgs::Pose& pose,
                std::string& class_name
            );
            StartGateWatershed();
    };
}

#endif