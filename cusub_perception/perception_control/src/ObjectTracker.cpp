#include "perception_control/ObjectTracker.h"

using namespace perception_control;

ObjectTracker::ObjectTracker(BoundingBox &box, const cv::Mat &image) :
    m_pointTracker(new KLTPointTracker()),
    m_boundingBox(box),
    m_valid(false),
    m_fixPoints(true)
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
        result.transform.transformBox(m_boundingBox);
        if (m_fixPoints)
        {
            m_boundingBox.fixBox(m_pointTracker->currentPoints());
        }
        m_valid = true;
    }
    else
    {
        m_valid = false;
    }

}
