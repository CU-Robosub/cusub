#include "tracking/Tracking.h"

using namespace tracking;

#include <iostream>
#include <opencv2/highgui.hpp>

Tracking::Tracking() :
    m_debugMode(false)
{}

void Tracking::onInit()
{
    ros::NodeHandle nhPrivate = getPrivateNodeHandle();
    nhPrivate.getParam("image_topic", m_imageTopicName);
    nhPrivate.getParam("detection_topic", m_detectionTopicName);
    nhPrivate.getParam("detection_thresh", m_detectionThresh);
    nhPrivate.getParam("debug_mode", m_debugMode);

    NODELET_INFO(m_imageTopicName.c_str());
    NODELET_INFO(m_detectionTopicName.c_str());
    if(m_debugMode) NODELET_INFO("true");
    else NODELET_INFO("false");

    m_frameCount = 0;

    m_nh = getNodeHandle();
    setupPublishers();
    setupSubscribers();

    NODELET_INFO("Tracking initialized");
}

Tracking::~Tracking()
{
    for (std::pair<std::string, ObjectTracker *> iter : m_objectMap)
    {
        delete iter.second;
    }
}

void Tracking::setupPublishers()
{
    m_debugPublisher = m_nh.advertise<sensor_msgs::Image>("tracking_debug", 1);
}
void Tracking::setupSubscribers()
{
    m_imageSubscriber = m_nh.subscribe(m_imageTopicName, 1, &Tracking::imageCallback, this);
    m_detectionSubscriber = m_nh.subscribe(m_detectionTopicName, 1, &Tracking::darknetCallback, this);
}

BoundingBox Tracking::getBox(const std::string &classname)
{
    if (m_objectMap.count(classname) != 0)
    {
        ObjectTracker * tracker = m_objectMap[classname];
        if (tracker->isValid())
        {
            return tracker->currentBox();
        }
    }

    return BoundingBox();
}

void Tracking::darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &bbs)
{
    ImageData image = ImageData(bbs->image);
    for(darknet_ros_msgs::BoundingBox box : bbs->bounding_boxes)
    {
        if (box.probability > m_detectionThresh)
        {
            std::string classname = box.Class;
            BoundingBox bbox(box.xmin, box.ymin, box.xmax, box.ymax);
            objectDetected(classname, bbox, image);
        }
    }
}

void Tracking::imageCallback(const sensor_msgs::Image::ConstPtr &rosImage)
{
    ImageData image(rosImage);
    newImage(image);
    
    m_frameCount++;
}

void Tracking::objectDetected(const std::string &classname, BoundingBox &box, const ImageData &image)
{
    if (m_objectMap.count(classname) != 0)
    {
        ObjectTracker * objectTracker = m_objectMap[classname];
        if (objectTracker->isValid() == false)
        {
            objectTracker->initialize(box, image);
        }
        else
        {
            // the detection box and the tracking box don't overlap at all, reset
            if (objectTracker->currentBox().overlapArea(box) < 0)
            {
                std::cout << "No overlap for class: " << classname << std::endl;
                objectTracker->initialize(box, image);
            }
        }
    }
    // first detection, initialize the tracker here
    else
    {
        std::cout << "New tracker for class: " << classname << std::endl;
        m_objectMap.insert(std::make_pair(classname, new ObjectTracker(box, image)));
    }
}

void Tracking::newImage(const ImageData &image)
{
    for (std::pair<std::string, ObjectTracker *> iter : m_objectMap)
    {
        iter.second->updateImage(image);
    }

    if (m_debugMode)
    {
        publishDebugBoxes();
    }
}

// todo SK: custom message and publish out
void Tracking::publishBoxes()
{
    for (std::pair<std::string, ObjectTracker *> iter : m_objectMap)
    {
        // todo SK
        // check if boxes overlap by too much, make invalid
        // need to check all permutations
    }
}

void Tracking::publishDebugBoxes()
{
    cv::Mat image = cv::Mat();
    for (std::pair<std::string, ObjectTracker *> iter : m_objectMap)
    {
        if (iter.second->isValid())
        {
            if (image.empty())
            {
                image = iter.second->currentImage().cvImage();
                cv::waitKey(10);
                cv::cvtColor(image, image, CV_RGB2BGR);
            }
            // get and draw the most recent box
            BoundingBox bbox = iter.second->currentBox();
            cv::rectangle(image, bbox.roiRect(), cv::Scalar(0,0,255), 2);
        }
    }

    sensor_msgs::Image imageMsg = ImageData::cvImagetoROS(image);
    m_debugPublisher.publish(imageMsg);
}

PLUGINLIB_EXPORT_CLASS(tracking::Tracking, nodelet::Nodelet);