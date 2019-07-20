#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <opencv2/core.hpp>

namespace perception_control
{

class BoundingBox
{

public:
    BoundingBox(const int &xmin, const int &ymin, const int &xmax, const int &ymax);

    void setPoints(const std::vector<cv::Point2f> &points);

    int xmin() const;
    int ymin() const;
    int xmax() const;
    int ymax() const;

    std::vector<cv::Point2f> cornerPoints() const;
    cv::Rect roiRect() const;

private:
    int m_xmin;
    int m_ymin;
    int m_xmax;
    int m_ymax;

};
    
}; // namespace perception_control


#endif // BOUNDINGBOX_H