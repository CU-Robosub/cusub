#ifndef DETECTION_TREE_SRC_DETECTION_TREE_H_
#define DETECTION_TREE_SRC_DETECTION_TREE_H_

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Header.h>
#include <string>
#include <vector>
#include <detection_tree_msgs/Dvector.h>

typedef struct{
    int num; // dobject number
    std::string class_id;
    geometry_msgs::Pose pose;
    std::vector<detection_tree_msgs::Dvector*> dvector_list;
} Dobject;

void getLastDvectors(Dobject* dobj, int num, std::vector<detection_tree_msgs::Dvector*>& dvs)
{
    int size = dobj->dvector_list.size();
    if( num > size )
        num = size;
    for( int i=size-1; i >= size - num; i--)
    {
        dvs.push_back(dobj->dvector_list[i]);
    }
}

#endif