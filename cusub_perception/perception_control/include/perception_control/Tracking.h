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
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "ObjectTracker.h"


namespace perception_control
{

class Tracking : public nodelet::Nodelet
{
public:
    virtual void onInit();

    BoundingBox getBox(const std::string &classname);

    void darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& bbs);
    // void occamCallback();

    void objectDetected(const std::string &classname, BoundingBox &box, const cv::Mat &image);
    void newImage(const cv::Mat &image);


private:
    ~Tracking();
    
    std::map<std::string, ObjectTracker *> m_objectMap;


};

}; // namespace perception_control

#endif // TRACKING_H