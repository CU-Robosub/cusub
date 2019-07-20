#include "perception_control/BoundingBox.h"

using namespace perception_control;

BoundingBox::BoundingBox(const int &xmin, const int &ymin, const int &xmax, const int &ymax) :
    m_xmin(xmin),
    m_ymin(ymin),
    m_xmax(xmax),
    m_ymax(ymax)
{}

// order will be topLeft, topRight, bottomLeft, bottomRight
void BoundingBox::setPoints(const std::vector<cv::Point2f> &points)
{
    assert(points.size() == 4);

    m_xmin = static_cast<int>(points.at(0).x);
    m_ymin = static_cast<int>(points.at(0).y);
    
    m_xmax = static_cast<int>(points.at(2).x);
    m_ymax = static_cast<int>(points.at(2).y);
}

int BoundingBox::xmin() const
{
    return m_xmin;
}

int BoundingBox::ymin() const
{
    return m_ymin;
}

int BoundingBox::xmax() const
{
    return m_xmax;
}

int BoundingBox::ymax() const
{
    return m_ymax;
}

cv::Rect BoundingBox::roiRect() const
{
    return cv::Rect(m_xmin, m_ymin, (m_xmax - m_xmin), (m_ymax - m_ymin));
}

std::vector<cv::Point2f> BoundingBox::cornerPoints() const
{
    cv::Point2f topLeft(static_cast<float>(m_xmin), static_cast<float>(m_ymin));
    cv::Point2f topRight(static_cast<float>(m_xmax), static_cast<float>(m_ymin));
    cv::Point2f bottomLeft(static_cast<float>(m_xmin), static_cast<float>(m_ymax));
    cv::Point2f bottomRight(static_cast<float>(m_xmax), static_cast<float>(m_ymax));

    return std::vector<cv::Point2f>{topLeft, topRight, bottomLeft, bottomRight};
}
