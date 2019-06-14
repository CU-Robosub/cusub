/*
    PG used to ignore darknet boxes of unwanted classes.
 */

#ifndef IGNORE_PG_CLASS_SRC_IGNORE_CLASS_H_
#define IGNORE_PG_CLASS_SRC_IGNORE_CLASS_H_

#include <localizer/pose_generator.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <geometry_msgs/Pose.h>

namespace pose_generator
{
    class IgnorePG : public PoseGenerator
    {
        public:
        bool generatePose(
                sensor_msgs::Image& image, 
                std::vector<darknet_ros_msgs::BoundingBox>& bbs,
                geometry_msgs::Pose& pose,
                std::string& class_name
            ){return false;}
    };
}

#endif