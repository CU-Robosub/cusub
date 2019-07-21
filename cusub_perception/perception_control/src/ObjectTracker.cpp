#include "perception_control/ObjectTracker.h"

using namespace perception_control;

ObjectTracker::ObjectTracker(BoundingBox &box, const cv::Mat &image) :
    m_pointTracker(new KLTPointTracker()),
    m_boundingBox(box),
    m_valid(false)
{
    initialize(box, image);
}

void ObjectTracker::initialize(BoundingBox &box, const cv::Mat &image)
{
    PointTracker::Result result = m_pointTracker->initialize(image, box);
    m_valid = result.status == PointTracker::STATUS::Success;
    m_boundingBox = box;
}

BoundingBox ObjectTracker::getCurrentBox() const
{
    return m_boundingBox;
}

bool ObjectTracker::isValid()
{
    return m_valid;
}

void ObjectTracker::updateImage(const cv::Mat &image)
{
    PointTracker::Result result = m_pointTracker->trackPoints(image);

    if (result.status == PointTracker::STATUS::Success)
    {
        m_boundingBox.setTransform(result.transform);
        m_valid = true;
    }
    else
    {
        m_valid = false;
    }

}
