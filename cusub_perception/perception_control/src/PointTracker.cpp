#include "perception_control/PointTracker.h"

using namespace perception_control;

bool PointTracker::isValid()
{
    return m_currentPoints.size() > 2;
}

std::vector<cv::Point2f> PointTracker::currentPoints() const
{
    return m_currentPoints;
}