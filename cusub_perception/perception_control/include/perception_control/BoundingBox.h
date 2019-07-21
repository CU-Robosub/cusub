#ifndef BOUNDINGBOX_H
#define BOUNDINGBOX_H

#include <opencv2/core.hpp>
#include "AffineTransform.h"

namespace perception_control
{

class BoundingBox
{

public:
    BoundingBox();
    BoundingBox(const int &xmin, const int &ymin, const int &xmax, const int &ymax);

    void setTransform(const AffineTransform &tform);
    
    int xmin() const;
    int ymin() const;
    int xmax() const;
    int ymax() const;
    bool valid() const;
    std::vector<cv::Point2f> cornerPoints() const;
    cv::Rect roiRect() const;

private:
    std::vector<cv::Point2f> m_points;
    bool m_valid;

    int m_xmin;
    int m_xmax;
    int m_ymin;
    int m_ymax;

};
    
}; // namespace perception_control


#endif // BOUNDINGBOX_H