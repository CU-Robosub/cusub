#ifndef POSE_GENERATOR_CLASS_SRC_POSE_GENERATOR_CLASS_H_
#define POSE_GENERATOR_CLASS_SRC_POSE_GENERATOR_CLASS_H_

#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Scalar.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <localizer/Detection.h>

using namespace cv;
using namespace std;

namespace pose_generator
{
    class PoseGenerator
    {
        public:
            virtual bool generatePose(
                const sensor_msgs::Image& image, 
                const vector<darknet_ros_msgs::BoundingBox>& bbs,
                vector<localizer::Detection>& detections
            ){;}
            std::string pose_gen_name;

        protected:
            void getPoseFromPoints(vector<Point3f>& truth_pts, vector<Point2f>& img_points, geometry_msgs::Pose& pose);

            vector<double> downcam_camera_matrix_values{
                333.16354886231323, 0.0, 640.5, 0.0, 333.16354886231323, 480.5, 0.0, 0.0, 1.0
            };
            vector<double> downcam_dist_coefs_values{
               0.0, 0.0, 0.0, 0.0, 0.0
            };
            Mat downcam_camera_matrix{3,3,DataType<double>::type, downcam_camera_matrix_values.data()};
            Mat downcam_dist_coefs{4,1, DataType<double>::type, downcam_dist_coefs_values.data()};

            vector<double> occam_camera_matrix_values{
                656.911402, 0.000000, 373.735385, 0.000000, 719.001469, 156.944858, 0.000000, 0.000000, 1.000000
            };
            vector<double> occam_dist_coefs_values{
                -0.360085, 0.115116, 0.007768, 0.004616, 0.000000   
            };
            Mat occam_camera_matrix{3,3,DataType<double>::type, occam_camera_matrix_values.data()};
            Mat occam_dist_coefs{4,1, DataType<double>::type, occam_dist_coefs_values.data()};

            void cuprint(std::string str);
            void cuprint_warn(std::string str);
    };
}

#endif