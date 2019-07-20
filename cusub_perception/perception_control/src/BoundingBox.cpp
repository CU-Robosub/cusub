#include "perception_control/BoundingBox.h"

using namespace perception_control;

BoundingBox::BoundingBox() :
    m_valid(false)
{}

BoundingBox::BoundingBox(const int &xmin, const int &ymin, const int &xmax, const int &ymax) :
    m_xmin(xmin),
    m_ymin(ymin),
    m_xmax(xmax),
    m_ymax(ymax),
    m_valid(true)
{}

// order will be topLeft, topRight, bottomLeft, bottomRight
void BoundingBox::setPoints(const std::vector<cv::Point2f> &points)
{
    assert(points.size() == 4);

    m_xmin = static_cast<int>(points.at(0).x);
    m_ymin = static_cast<int>(points.at(0).y);
    
    m_xmax = static_cast<int>(points.at(3).x);
    m_ymax = static_cast<int>(points.at(3).y);
}

void BoundingBox::fixBox(const std::vector<cv::Point2f> &points)
{
    for (const cv::Point2f &pt : points)
    {
        int x = static_cast<int>(pt.x);
        int y = static_cast<int>(pt.y);
        if (x < m_xmin)
        {
            m_xmin = x;
        }
        else if (x > m_xmax)
        {
            m_xmax = x;
        }

        if (y < m_ymin)
        {
            m_ymin = y;
        }
        else if (y > m_ymax)
        {
            m_ymax = y;
        }
    }
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
