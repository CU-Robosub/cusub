#include "tracking/AffineTransform.h"

using namespace tracking;

AffineTransform::AffineTransform()
{
}

AffineTransform::AffineTransform(const AffineTransform &other)
{
    m_transform = other.m_transform;
}

AffineTransform::AffineTransform(const std::vector<cv::Point2f> &fromPts, const std::vector<cv::Point2f> &toPts, bool &success)
{
    // old version is less rigid, tends to increase error
    // m_transform = cv::findHomography(fromPts, toPts, CV_RANSAC);
    m_transform = cv::estimateAffinePartial2D(fromPts, toPts);
    success = m_transform.empty() == false;
}

void AffineTransform::transformPoints(std::vector<cv::Point2f> &pts) const
{
//    cv::perspectiveTransform(pts, pts, m_transform);
    // extend rigid transformation to use perspectiveTransform:
    cv::Mat H = cv::Mat(3,3,m_transform.type());
    H.at<double>(0,0) = m_transform.at<double>(0,0);
    H.at<double>(0,1) = m_transform.at<double>(0,1);
    H.at<double>(0,2) = m_transform.at<double>(0,2);

    H.at<double>(1,0) = m_transform.at<double>(1,0);
    H.at<double>(1,1) = m_transform.at<double>(1,1);
    H.at<double>(1,2) = m_transform.at<double>(1,2);

    H.at<double>(2,0) = 0.0;
    H.at<double>(2,1) = 0.0;
    H.at<double>(2,2) = 1.0;

    // compute perspectiveTransform on p1
    cv::perspectiveTransform(pts, pts, H);
}
