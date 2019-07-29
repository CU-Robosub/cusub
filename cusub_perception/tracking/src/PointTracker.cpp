#include "tracking/PointTracker.h"

using namespace tracking;

bool PointTracker::isValid()
{
    return m_currentPoints.size() > 2;
}

std::vector<cv::Point2f> PointTracker::currentPoints() const
{
    return m_currentPoints;
}

cv::Mat PointTracker::currentImage() const
{
    return m_currentImg;
}