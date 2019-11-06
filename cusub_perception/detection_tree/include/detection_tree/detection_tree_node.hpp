#ifndef DETECTION_TREE_NODE_SRC_DETECTION_TREE_NODE_H_
#define DETECTION_TREE_NODE_SRC_DETECTION_TREE_NODE_H_

#include <pluginlib/class_list_macros.h>
#include "detection_tree/detection_tree.hpp"
#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <vector>
// #include <map>
#include <string>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/CameraInfo.h>

// Camel case classes + functions, underscore variables + namespaces

namespace det_tree_ns
{
    class DetectionTree : public nodelet::Nodelet
    {
        public:
            virtual void onInit();
        private:
            std::string sub_name;
            void darknetCallback(const darknet_ros_msgs::BoundingBoxesPtr bbs);
            void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr ci);
            dvector* createDvector(darknet_ros_msgs::BoundingBox& bb, std_msgs::Header& image_header);
            bool boxToBearing(std_msgs::Header& camera_header, darknet_ros_msgs::BoundingBox& box, geometry_msgs::Quaternion& bearing);
            int determineDobject(dvector* dv_ptr, dobject* dobj_ptr); // -1 for new dobject
            void createDobject(dvector* dv_ptr);
            void addDvectorToDobject(dvector* dv_ptr, dobject* dobj_ptr);
            bool poseSolveDobject(dobject* dobj, geometry_msgs::Pose& pose);
            std::vector<dobject> dobject_list;
            ros::Subscriber darknet_sub;
            ros::Publisher dvector_pub;
            tf::TransformListener listener;

            bool all_info_topics_received;
            std::vector<ros::Subscriber> camera_info_subs;
            // some map between frame and coefficients
            std::vector<std::string> camera_info_topics = 
            {
               "/leviathan/cusub_common/occam/image0/camera_info",
               "/leviathan/cusub_common/occam/image1/camera_info",
               "/leviathan/cusub_common/occam/image2/camera_info",
               "/leviathan/cusub_common/occam/image3/camera_info",
               "/leviathan/cusub_common/occam/image4/camera_info",
               "/leviathan/cusub_common/downcam/camera_info"
            }; // TODO move to launch file for configurability
    };
}

#endif