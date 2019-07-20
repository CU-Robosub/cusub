#include "perception_control/Tracking.h"

using namespace perception_control;

void Tracking::onInit()
{}

Tracking::~Tracking()
{}

BoundingBox Tracking::getBox(const std::string &classname)
{
    if (m_objectMap.count(classname) != 0)
    {
        ObjectTracker * tracker = m_objectMap[classname];
        if (tracker->isValid())
        {
            return tracker->getCurrentBox();
        }
    }

    return BoundingBox();
}

void Tracking::darknetCallback(const darknet_ros_msgs::BoundingBoxesConstPtr &bbs)
{
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(bbs->image, sensor_msgs::image_encodings::RGB8);
    cv::Mat image = cv_ptr->image;
    for(darknet_ros_msgs::BoundingBox box : bbs->bounding_boxes)
    {
        std::string classname = box.Class;
        BoundingBox bbox(box.xmin, box.ymin, box.xmax, box.ymax);
        objectDetected(classname, bbox, image);
    }
}

void Tracking::objectDetected(const std::string &classname, BoundingBox &box, const cv::Mat &image)
{
    if (m_objectMap.count(classname) != 0)
    {
        // option to reinitialize. Maybe only for high probability?
    }

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
}