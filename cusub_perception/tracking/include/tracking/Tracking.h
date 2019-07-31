/**
 * @file tracking.cpp
 * @author Soroush Khadem (soroush.khadem@colorado.edu)
 * @brief Node to track objects
 * 
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
    explicit Tracking();
    virtual void onInit();

    BoundingBox getBox(const std::string &classname);

    void darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr& bbs);
    void imageCallback(const sensor_msgs::Image::ConstPtr &rosImage);

    void objectDetected(const std::string &classname, BoundingBox &box, const ImageData &image);
    void newImage(const ImageData &image);


    ~Tracking();
private:

    void setupPublishers();
    void setupSubscribers();
    
    void publishBoxes();
    void publishDebugBoxes();

    ros::NodeHandle m_nh;
    std::string m_imageTopicName;
    ros::Subscriber m_imageSubscriber;
    std::string m_detectionTopicName;
    ros::Subscriber m_detectionSubscriber;
    ros::Publisher m_debugPublisher;

    bool m_debugMode;
    uint m_frameCount;
    float m_detectionThresh;
    float m_reseedThresh;


    std::map<std::string, ObjectTracker *> m_objectMap;

};

}; // namespace perception_control

#endif // TRACKING_H