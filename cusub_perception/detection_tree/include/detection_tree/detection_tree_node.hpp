#ifndef DETECTION_TREE_NODE_SRC_DETECTION_TREE_NODE_H_
#define DETECTION_TREE_NODE_SRC_DETECTION_TREE_NODE_H_

#include <pluginlib/class_list_macros.h>
#include "detection_tree/detection_tree.hpp"
// #include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <vector>
// #include <map>
// #include <string>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <std_msgs/Header.h>
#include <geometry_msgs/Pose.h>

namespace det_tree_ns
{
    class DetectionTree
    {
        public:
            virtual void onInit();
        private:
            void darknetCallback(const darknet_ros_msgs::BoundingBoxesPtr bbs);
            void createDvector(darknet_ros_msgs::BoundingBox& bb, std_msgs::Header& image_header, dvector& dv);
            void createDobject(dvector dv, int dobject_num);
            void determineDobject(dvector dv);
            bool pose_solve(dobject& dobj, geometry_msgs::Pose& pose);
            vector<dobject> dobject_list;
            ros::Subscriber darknet_sub;
            // transform listener
    }
}

#endif