/**
 * @file tracking.cpp
 * @author Soroush Khadem (soroush.khadem@colorado.edu)
 * @brief 
 * @date 2019-07-17
 */

#ifndef TRACKING_H
#define TRACKING_H

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "KLTPointTracker.h"

namespace perception_control
{

class Tracking : public nodelet::Nodelet
{
public:
    virtual void onInit();
private:
    ~Tracking();
    
};

}; // namespace perception_control

#endif // TRACKING_H