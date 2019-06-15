#ifndef START_GATE_WATERSHED_CLASS_SRC_START_GATE_WATERSHED_CLASS_H_
#define START_GATE_WATERSHED_CLASS_SRC_START_GATE_WATERSHED_CLASS_H_

#include <nodelet/nodelet.h>
#include <localizer/pose_generator.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <localizer/Detection.h>

// Testing of Watershed Algorithm
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Quaternion.h>
// #include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
/* 1-------3    SolvePnp Points
   |       |      
   |   X   |
   0       2
 */
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
        private:
            void getPoseFromVectors(Mat rvec, Mat tvec, geometry_msgs::Pose& pose);
            void sortBoxes(vector<darknet_ros_msgs::BoundingBox>& bbs);
            bool getPoints(Mat& img, vector<Point2f>& points);
            vector<Point3f> gate_truth_pts{
                Point3f(0,-1.6,-0.6),
                Point3f(0,-1.6, 0.6),
                Point3f(0, 1.6,-0.6),
                Point3f(0, 1.6, 0.6)
            };
            vector<double> occam_camera_matrix_values{
                656.911402, 0.000000, 373.735385, 0.000000, 719.001469, 156.944858, 0.000000, 0.000000, 1.000000
            };
            vector<double> occam_dist_coefs_values{
                -0.360085, 0.115116, 0.007768, 0.004616, 0.000000   
            };
            Mat occam_camera_matrix{3,3,DataType<double>::type, occam_camera_matrix_values.data()};
            Mat occam_dist_coefs{4,1, DataType<double>::type, occam_dist_coefs_values.data()};
    };
}

#endif