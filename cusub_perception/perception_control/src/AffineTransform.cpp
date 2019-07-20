#include "perception_control/AffineTransform.h"

using namespace perception_control;

AffineTransform::AffineTransform()
{
}

AffineTransform::AffineTransform(const AffineTransform &other)
{
    m_transform = other.m_transform;
}

AffineTransform::AffineTransform(const std::vector<cv::Point2f> &fromPts, const std::vector<cv::Point2f> &toPts)
{
    // todo SK: this doesn't give a great transform
    // m_transform = cv::getPerspectiveTransform(fromPts, toPts);
    m_transform = cv::estimateRigidTransform(fromPts, toPts, false);
    // m_transform = cv::getAffineTransform(fromPts, toPts);
}

void AffineTransform::transformPoints(std::vector<cv::Point2f> &pts)
{
    cv::transform(pts, pts, m_transform);
}

void AffineTransform::transformBox(BoundingBox &box)
{
    std::vector<cv::Point2f> pts = box.cornerPoints();
    transformPoints(pts);
    box.setPoints(pts);
}