/**
 * @file tracking.cpp
 * @author Soroush Khadem (soroush.khadem@colorado.edu)
 * @brief 
 * @date 2019-07-17
 */

#ifndef TRACKING_H
#define TRACKING_H

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>
#include <cv_bridge/cv_bridge.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <sensor_msgs/Image.h>
#include "ObjectTracker.h"


namespace perception_control
{

class Tracking : public nodelet::Nodelet
{
public:
    virtual void onInit();

    BoundingBox getBox(const std::string &classname);

    void darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& bbs);
    void imageCallback(const sensor_msgs::Image::ConstPtr &image);

    void objectDetected(const std::string &classname, BoundingBox &box, const cv::Mat &image);
    void newImage(const cv::Mat &image);


private:
    ~Tracking();

    void setupPublishers();
    void setupSubscribers();
    
    void publishBoxes();
    void publishDebugBoxes();

    cv::Mat rosImageToCV(const sensor_msgs::Image::ConstPtr &image);
    sensor_msgs::Image cvImagetoROS(const cv::Mat &image);

    ros::NodeHandle m_nh;
    ros::Subscriber m_imageSubscriber;
    ros::Subscriber m_darknetSubscriber;
    ros::Publisher m_debugPublisher;

    std::map<std::string, ObjectTracker *> m_objectMap;

    bool m_debugMode;

    uint m_frameCount;
};

}; // namespace perception_control

#endif // TRACKING_H