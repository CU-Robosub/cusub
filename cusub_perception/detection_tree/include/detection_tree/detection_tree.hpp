#ifndef DETECTION_TREE_SRC_DETECTION_TREE_H_
#define DETECTION_TREE_SRC_DETECTION_TREE_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <string>
#include <vector>

typedef struct{
    geometry_msgs::PoseStamped det_pose;
    std_msgs::Header camera_header;
    std::string class_id;
    int dobject_num;
    float probability;
} dvector;

typedef struct{
    int dobject_num;
    std::string class_id;
    geometry_msgs::Pose pose;
    std::vector<dvector> dvector_list;
} dobject;

#endif