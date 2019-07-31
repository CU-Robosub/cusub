#include "tracking/BoundingBox.h"

using namespace perception_control;

// in pixels
const int BoundingBox::AREA_THRESHOLD = 100;
const int BoundingBox::LOW_THRESHOLD = -25; 
const int BoundingBox::HIGH_THRESHOLD = 1000;

BoundingBox::BoundingBox() :
    m_valid(false)
{}

BoundingBox::BoundingBox(const int &xmin, const int &ymin, const int &xmax, const int &ymax) :
    m_xmin(xmin),
    m_ymin(ymin),
    m_xmax(xmax),
    m_ymax(ymax),
    m_probability(-1),
    m_valid(true)
{
    m_points = cornerPoints();
}

BoundingBox::BoundingBox(const int &xmin, const int &ymin, const int &xmax, const int &ymax, const float &probability) :
    m_xmin(xmin),
    m_ymin(ymin),
    m_xmax(xmax),
    m_ymax(ymax),
    m_probability(probability),
    m_valid(true)
{
    m_points = cornerPoints();
}


void BoundingBox::setTransform(const AffineTransform &tform)
{
    tform.transformPoints(m_points);

    m_xmin = static_cast<int>(m_points.at(0).x);
    m_ymin = static_cast<int>(m_points.at(0).y);
    m_xmax = static_cast<int>(m_points.at(3).x);
    m_ymax = static_cast<int>(m_points.at(3).y);

}

bool BoundingBox::isValid() const
{
    return m_valid;
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

float BoundingBox::probability() const
{
    return m_probability;
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

bool BoundingBox::checkBox()
{
    bool retVal = true;

    if (m_xmin < LOW_THRESHOLD || m_ymin < LOW_THRESHOLD || m_xmax < LOW_THRESHOLD || m_ymax < LOW_THRESHOLD)
    {
        retVal = false;
    }

    if (m_xmin > HIGH_THRESHOLD || m_ymin > HIGH_THRESHOLD || m_xmax > HIGH_THRESHOLD || m_ymax > HIGH_THRESHOLD)
    {
        retVal = false;
    }

    if (roiRect().area() < AREA_THRESHOLD)
    {
        retVal = false;
    }

    return retVal;
}

int BoundingBox::area() const
{
    return roiRect().area();
}

int BoundingBox::overlapArea(const BoundingBox &other) const
{
    int area;
    if (m_xmin > other.xmax() || m_xmax < other.xmin())
    {
        area = -1;
    }
    else if (m_ymin > other.ymax() || m_ymax < other.ymin())
    {
        area = -1;
    }
    // do intersect
    else
    {
        area = 1;
    }

    return area;
}