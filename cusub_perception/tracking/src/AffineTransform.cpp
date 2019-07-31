#include "tracking/AffineTransform.h"

using namespace perception_control;

AffineTransform::AffineTransform()
{
}

AffineTransform::AffineTransform(const AffineTransform &other)
{
    m_transform = other.m_transform;
}

AffineTransform::AffineTransform(const std::vector<cv::Point2f> &fromPts, const std::vector<cv::Point2f> &toPts, bool &success)
{
    m_transform = cv::findHomography(fromPts, toPts, CV_RANSAC);
    success = m_transform.empty() == false;
}

void AffineTransform::transformPoints(std::vector<cv::Point2f> &pts) const
{
    cv::perspectiveTransform(pts, pts, m_transform);
}