#ifndef POSE_GENERATOR_CLASS_SRC_POSE_GENERATOR_CLASS_H_
#define POSE_GENERATOR_CLASS_SRC_POSE_GENERATOR_CLASS_H_

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <vector>

namespace pose_generator
{
    class PoseGenerator
    {
        public:
            virtual bool generatePose(
                sensor_msgs::Image& image, 
                std::vector<darknet_ros_msgs::BoundingBox>& bbs,
                geometry_msgs::Pose& pose,
                std::string& class_name
            ){;}
    };
}

#endif