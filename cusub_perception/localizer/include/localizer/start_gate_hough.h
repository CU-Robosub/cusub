#ifndef START_GATE_HOUGH_CLASS_SRC_START_GATE_HOUGH_CLASS_H_
#define START_GATE_HOUGH_CLASS_SRC_START_GATE_HOUGH_CLASS_H_

#include <cstdlib>
#include <nodelet/nodelet.h>
#include <localizer/pose_generator.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <localizer/Detection.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Testing
#include <opencv2/highgui/highgui.hpp>

/* 1-------3    SolvePnp Points
   |       |      
   |   X   |
   0       2
 */
using namespace cv;
using namespace std;

namespace pose_generator
{
    class StartGateHough : public PoseGenerator
    {
        public:
            bool generatePose(
                sensor_msgs::Image& image, 
                vector<darknet_ros_msgs::BoundingBox>& bbs,
                geometry_msgs::Pose& pose,
                string& class_name
            );
            StartGateHough();
        private:
            void sortBoxes(vector<darknet_ros_msgs::BoundingBox>& bbs);
            bool getPoints(Mat& img, vector<Point2f>& points);
            void publishLegSide(vector<darknet_ros_msgs::BoundingBox>& bbs);
            ros::Publisher small_leg_side_pub; // left or right
            vector<Point3f> gate_truth_pts{
                Point3f(0,-1.6,-0.6),
                Point3f(0,-1.6, 0.6),
                Point3f(0, 1.6,-0.6),
                Point3f(0, 1.6, 0.6)
            };
            bool three_legs;
    };
}

#endif