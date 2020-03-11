#ifndef JIANGSHI_WATERSHED_CLASS_SRC_JIANGSHI_WATERSHED_CLASS_H_
#define JIANGSHI_WATERSHED_CLASS_SRC_JIANGSHI_WATERSHED_CLASS_H_

#include <localizer/pose_generator.h>
#include <nodelet/nodelet.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace pose_generator
{
    class JiangshiWatershed : public PoseGenerator
    {
        public:
            JiangshiWatershed();
            bool generatePose(
                const sensor_msgs::Image& image, 
                const vector<darknet_ros_msgs::BoundingBox>& bbs,
                vector<localizer_msgs::Detection>& detections
            );
        private:
            void getOrientationFromAspectRatio(const darknet_ros_msgs::BoundingBox& bb, float horizontalDist, geometry_msgs::Quaternion& quat);
            bool getPoints(Mat& img, int border_size, vector<Point2f>& points);
            void sortPoints(Mat& img, vector<Point2f>& points);
            bool checkBoxes(const vector<darknet_ros_msgs::BoundingBox>& bbs, int border_size);
            vector<Point3f> truth_pts{      // TODO lookup the actual dimensions of jiangshi
                Point3f(0,-0.3048,-0.61595),
                Point3f(0,-0.3048, 0.61595),
                Point3f(0, 0.3048,-0.61595),
                Point3f(0, 0.3048, 0.61595)
            };
            bool useAspectRatio;
            float aspectRatio90Deg;
    };
}

#endif
