#include "perception_control/Tracking.h"

using namespace perception_control;

void Tracking::onInit()
{
    m_debugMode = true; // todo SK: read from param
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
    // todo SK: ns issues
    m_imageSubscriber = m_nh.subscribe("/leviathan/cusub_common/occam/image0", 1, &Tracking::imageCallback, this);
    m_darknetSubscriber = m_nh.subscribe("/leviathan/cusub_perception/darknet_ros/bounding_boxes", 1, &Tracking::darknetCallback, this);
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
    // todo SK: convert sensor_msgs::Image to ConstPtr
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(bbs->image, sensor_msgs::image_encodings::RGB8);
    cv::Mat image = cv_ptr->image;
    for(darknet_ros_msgs::BoundingBox box : bbs->bounding_boxes)
    {
        std::string classname = box.Class;
        BoundingBox bbox(box.xmin, box.ymin, box.xmax, box.ymax);
        objectDetected(classname, bbox, image);
    }
}

void Tracking::imageCallback(const sensor_msgs::Image::ConstPtr &image)
{
    cv::Mat cvImage = rosImageToCV(image);
    newImage(cvImage);
    
    m_frameCount++;
}

void Tracking::objectDetected(const std::string &classname, BoundingBox &box, const cv::Mat &image)
{
    if (m_objectMap.count(classname) != 0)
    {
        // option to reinitialize. Maybe only for high probability?
    }
    // first detection, initialize tracker here
    else
    {
        m_objectMap.insert(std::make_pair(classname, new ObjectTracker(box, image)));
    }
}

void Tracking::newImage(const cv::Mat &image)
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
{}

void Tracking::publishDebugBoxes()
{
    cv::Mat image = cv::Mat();
    for (std::pair<std::string, ObjectTracker *> iter : m_objectMap)
    {
        if (image.empty())
        {
            image = iter.second->currentImage();
            cv::cvtColor(image, image, CV_GRAY2BGR);
        }
        // get and draw the most recent box
        BoundingBox bbox = iter.second->currentBox();
        cv::rectangle(image, bbox.roiRect(), cv::Scalar(0,0,255), 2);
    }

    sensor_msgs::Image imageMsg = cvImagetoROS(image);
    m_debugPublisher.publish(imageMsg);
}

cv::Mat Tracking::rosImageToCV(const sensor_msgs::Image::ConstPtr &image)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);

    return cv_ptr->image;
}

sensor_msgs::Image Tracking::cvImagetoROS(const cv::Mat &image)
{
    // create header
    std_msgs::Header header;
    header.seq = m_frameCount;
    header.stamp = ros::Time::now();

    // setup bridge
    cv_bridge::CvImage imgBridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::BGR8, image);
    
    // convert
    sensor_msgs::Image imgMsg;
    imgBridge.toImageMsg(imgMsg);

    return imgMsg;
}

PLUGINLIB_EXPORT_CLASS(perception_control::Tracking, nodelet::Nodelet);