/*
    PG used to ignore certain darknet bb box classes.

    Used to not get the error:
    "No pose generator given for"
 */

#ifndef IGNORE_PG_CLASS_SRC_IGNORE_CLASS_H_
#define IGNORE_PG_CLASS_SRC_IGNORE_CLASS_H_

#include <localizer/pose_generator.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <geometry_msgs/Pose.h>

using namespace std;

namespace pose_generator
{
    class IgnorePG : public PoseGenerator
    {
        public:
        bool generatePose(
                const sensor_msgs::Image& image, 
                const vector<darknet_ros_msgs::BoundingBox>& bbs,
                vector<localizer::Detection>& detections
            ){ return false;}
    };
}

#endif