#ifndef DETECTION_TREE_SRC_DETECTION_TREE_H_
#define DETECTION_TREE_SRC_DETECTION_TREE_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <string>
#include <vector>
#include <detection_tree/Dvector.h>

typedef struct{
    int dobject_num;
    std::string class_id;
    geometry_msgs::Pose pose;
    std::vector<detection_tree::Dvector> dvector_list;
} Dobject;

#endif