#ifndef DETECTION_TREE_SRC_DETECTION_TREE_H_
#define DETECTION_TREE_SRC_DETECTION_TREE_H_

#include <geometry_msgs/Pose.h>
#include <std_msgs/Header.h>
#include <string>
#include <vector>

typedef struct{
    float azimuth;
    float bearing;
    geometry_msgs::Pose camera_pose;
    std_msgs::Header header;
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