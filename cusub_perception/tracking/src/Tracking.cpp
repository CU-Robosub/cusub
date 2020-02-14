#include "tracking/Tracking.h"

using namespace tracking;

#include <iostream>
#include <opencv2/highgui.hpp>


const int OVERLAP_THRESH = 0.5; // in percent
const int COFFIN_OVERLAP_AMNT = 100; // each side

// called from unit test
Tracking::Tracking() :
    m_debugMode(false),
    m_publishBoxes(false)
{}

// called from nodelet
void Tracking::onInit()
{
    ros::NodeHandle nhPrivate = getPrivateNodeHandle();
    nhPrivate.getParam("image_topic", m_imageTopicName);
    nhPrivate.getParam("detection_topic", m_detectionTopicName);
    nhPrivate.getParam("detection_thresh", m_detectionThresh);
    nhPrivate.getParam("reseed_thresh", m_reseedThresh);
    nhPrivate.getParam("debug_mode", m_debugMode);
    

    NODELET_INFO("%s", m_imageTopicName.c_str());
    NODELET_INFO("%s", m_detectionTopicName.c_str());
    if(m_debugMode) NODELET_INFO("Debug: true");
    else NODELET_INFO("Debug: false");

    // initialize member variables
    m_nh = getNodeHandle();
    m_publishBoxes = true;
    m_frameCount = 0;
    setupPublishers();
    setupSubscribers();

    // setup configuration service
    m_thresholdServer = nhPrivate.advertiseService("tracking_threshold", &Tracking::configureTracking, this);

    NODELET_INFO("Tracking initialized");
}

Tracking::~Tracking()
{
    for (std::pair<std::string, std::vector<ObjectTracker *> > iter : m_objectMap)
    {
        for (ObjectTracker * objTracker : iter.second)
        {
            delete objTracker;
        }
    }
}

void Tracking::setupPublishers()
{
    m_debugPublisher = m_nh.advertise<sensor_msgs::Image>("tracking_debug", 1);
    m_bboxPublisher = m_nh.advertise<darknet_ros_msgs::BoundingBoxes>("tracking_boxes", 1);
}
void Tracking::setupSubscribers()
{
    m_imageSubscriber = m_nh.subscribe(m_imageTopicName, 1, &Tracking::imageCallback, this);
    m_detectionSubscriber = m_nh.subscribe(m_detectionTopicName, 1, &Tracking::darknetCallback, this);
}

bool Tracking::configureTracking(tracking::TrackingConfig::Request &req, tracking::TrackingConfig::Response &resp)
{
    m_detectionThresh = req.threshold;
    return true;
}

std::vector<BoundingBox> Tracking::getBoxes(const std::string &framename)
{
    std::vector<BoundingBox> boxes;
    if (m_objectMap.count(framename) != 0)
    {
        std::vector<ObjectTracker *> trackers = m_objectMap[framename];
        for (ObjectTracker * tracker : trackers)
        {
            if (tracker->isValid())
            {
                boxes.push_back(tracker->currentBox());
            }
        }
    }

    return boxes;
}

void Tracking::darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &bbs)
{
    ImageData image = ImageData(bbs->image);
    image.setFrameId(bbs->image_header.frame_id);
    for(darknet_ros_msgs::BoundingBox box : bbs->bounding_boxes)
    {
        if (box.probability > m_detectionThresh)
        {
            std::string classname = box.Class;
            BoundingBox bbox(box.xmin, box.ymin, box.xmax, box.ymax, box.probability);
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

void Tracking::objectDetected(const std::string &classname, BoundingBox &bbox, const ImageData &image)
{
    ObjectTracker * tracker = findTracker(image.frameId(), classname, bbox);

    if (tracker != nullptr)
    {   
        updateTracker(tracker, bbox, image);
    }
    else
    {
        // add the tracker
        if (classname == "coffin")
        {
            BoundingBox extendedBox = bbox;
            extendedBox.extendBox(COFFIN_OVERLAP_AMNT);
            m_objectMap[image.frameId()].push_back(new ObjectTracker(bbox, image, classname));
        }
        else
        {
            m_objectMap[image.frameId()].push_back(new ObjectTracker(bbox, image, classname));
        }
        std::cout << "new tracker for class: " << classname << " for frame: " << image.frameId() << std::endl;
    }
}

ObjectTracker * Tracking::findTracker(const std::string &frameId, const std::string &classname, const BoundingBox &bbox)
{
    ObjectTracker * objectTracker = nullptr;
    std::vector <ObjectTracker *> classTrackers;
    if (m_objectMap.count(frameId) != 0)
    {
        std::vector<ObjectTracker *> trackers = m_objectMap[frameId];
        for (ObjectTracker * tracker : trackers)
        {
            if (tracker->classname() == classname)
            {
                classTrackers.push_back(tracker);
            }
        }
    }

    int highestOverlap = -1e09;
    for (ObjectTracker * tracker : classTrackers)
    {
        int overlap = tracker->currentBox().overlapArea(bbox);
        if (overlap > highestOverlap)
        {
            objectTracker = tracker;
            highestOverlap = overlap;
        }
    }

    return objectTracker;
}

void Tracking::updateTracker(ObjectTracker * objectTracker,const BoundingBox &bbox, const ImageData &image)
{
    if (objectTracker->isValid() == false)
    {
        BoundingBox box = bbox;
        objectTracker->initialize(box, image);
    }
    else
    {
        // the detection box and the tracking box don't overlap at all, reset
        if (objectTracker->currentBox().overlapArea(bbox) < 0 || 
            bbox.probability() > m_reseedThresh)
        {
            // std::cout << "Overlap area for " << objectTracker->classname() << ": " << objectTracker->currentBox().overlapArea(bbox) << std::endl;
            BoundingBox box = bbox;
            objectTracker->initialize(box, image);
        }        
    }
}

void Tracking::newImage(const ImageData &image)
{
    for (ObjectTracker * tracker : m_objectMap[image.frameId()])
    {
        tracker->updateImage(image);
    }

    if (m_publishBoxes)
    {
        publishBoxes();
    }

    if (m_debugMode)
    {
        publishDebugBoxes();
    }
}

// todo SK
// check if boxes overlap by too much, make invalid
void Tracking::publishBoxes()
{
    // loop through all frames
    for (std::pair<std::string, std::vector<ObjectTracker *> > iter : m_objectMap)
    {
        bool first = true;
        darknet_ros_msgs::BoundingBoxes boundingBoxes;
        for (ObjectTracker * tracker : iter.second)
        {
            if (first)
            {
                // this is not correct. It will be out of sync between different trackers
                // Where should the image be held to avoid this problem / is it possible
                boundingBoxes.image = *tracker->currentImageData().rosImage().get();
                boundingBoxes.image_header = tracker->currentImageData().rosImage()->header;
                boundingBoxes.header = boundingBoxes.image_header;    
                first = false;
            }

            if (tracker->isValid())
            {
                darknet_ros_msgs::BoundingBox darknetBox;
                
                BoundingBox bbox = tracker->currentBox();
                darknetBox.Class = tracker->classname();
                darknetBox.probability = -1;
                darknetBox.xmin = bbox.xmin();
                darknetBox.xmax = bbox.xmax();
                darknetBox.ymin = bbox.ymin();
                darknetBox.ymax = bbox.ymax();

                boundingBoxes.bounding_boxes.push_back(darknetBox);
                
            }
        }

        m_bboxPublisher.publish(boundingBoxes);
    }
}

void Tracking::publishDebugBoxes()
{
    cv::Mat image = cv::Mat();
    for (std::pair<std::string, std::vector<ObjectTracker *> > iter : m_objectMap)
    {
        for (ObjectTracker * tracker : iter.second)
        {
            if (tracker->isValid())
            {
                if (image.empty())
                {
                    image = tracker->currentImage();
                    cv::cvtColor(image, image, CV_GRAY2BGR);
                }
                // get and draw the most recent box
                BoundingBox bbox = tracker->currentBox();
                if (bbox.xmax() - bbox.xmin() > image.size().width &&
                    bbox.ymax() - bbox.ymin() > image.size().height)
                    // collision detection example
                {
                    cv::rectangle(image, bbox.roiRect(), cv::Scalar(0,255,0), -1);
                }
                else
                {
                    cv::rectangle(image, bbox.roiRect(), cv::Scalar(0,0,255), 2);
                }
            }
        }
        sensor_msgs::Image imageMsg = ImageData::cvImagetoROS(image);
        m_debugPublisher.publish(imageMsg);
    }
}

PLUGINLIB_EXPORT_CLASS(tracking::Tracking, nodelet::Nodelet);