#ifndef Z_PLANE_CLASS_SRC_Z_PLANE_CLASS_H_
#define Z_PLANE_CLASS_SRC_Z_PLANE_CLASS_H_

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
#include <tf/transform_listener.h>

namespace pose_generator
{
    class ZPlane : public PoseGenerator
    {
        public:
            ZPlane();
            bool generatePose(
                sensor_msgs::Image& image, 
                vector<darknet_ros_msgs::BoundingBox>& bbs,
                geometry_msgs::PoseStamped& pose,
                string& class_name
            );
        protected:
            tf::TransformListener listener;
            geometry_msgs::Point transformPoint(
                std_msgs::Header header,
                std::string target_frame,
                double x,
                double y,
                double z
            );

    };
}

#endif
