#ifndef START_GATE_WATERSHED_CLASS_SRC_START_GATE_WATERSHED_CLASS_H_
#define START_GATE_WATERSHED_CLASS_SRC_START_GATE_WATERSHED_CLASS_H_

#include <nodelet/nodelet.h>
#include <localizer/pose_generator.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// Testing of Watershed Algorithm
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
using namespace cv;

using namespace std;

namespace pose_generator
{
    class StartGateWatershed : public PoseGenerator
    {
        public:
            bool generatePose(
                sensor_msgs::Image& image, 
                vector<darknet_ros_msgs::BoundingBox>& bbs,
                geometry_msgs::Pose& pose,
                string& class_name
            );
            void sortBoxes(vector<darknet_ros_msgs::BoundingBox>& bbs);
            bool getPoints(Mat& img, vector<Point>& points);
            StartGateWatershed();
    };
}

#endif