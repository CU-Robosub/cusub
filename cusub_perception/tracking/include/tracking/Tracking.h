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


namespace tracking
{

class Tracking : public nodelet::Nodelet
{
public:
    explicit Tracking();
    virtual void onInit();

    std::vector<BoundingBox> getBoxes(const std::string &classname);

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

    void updateTracker(ObjectTracker * objectTracker,const BoundingBox &bbox, const ImageData &image);
    ObjectTracker * findTracker(const std::string &frameId, const std::string &classname, const BoundingBox &bbox);

    ros::NodeHandle m_nh;
    ros::Publisher m_bboxPublisher;
    std::string m_imageTopicName;
    ros::Subscriber m_imageSubscriber;
    std::string m_detectionTopicName;
    ros::Subscriber m_detectionSubscriber;
    ros::Publisher m_debugPublisher;

    bool m_publishBoxes;
    bool m_debugMode;
    uint m_frameCount;
    float m_detectionThresh;
    float m_reseedThresh;


    std::map <std::string, std::vector<ObjectTracker*> > m_objectMap;

};

}; // namespace tracking

#endif // TRACKING_H